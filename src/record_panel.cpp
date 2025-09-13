#include "record_panel.hpp"

#include <algorithm>
#include <cmath>

#include <QApplication>
#include <QGuiApplication>
#include <QImage>
#include <QLabel>
#include <QPixmap>
#include <QScreen>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_rendering/render_window.hpp>  // correct header for RenderWindow

using namespace std::chrono_literals;

namespace rviz_record
{

// ---------- helpers ----------
static inline int make_even(int v) { return std::max(2, v & ~1); }

static rviz_common::RenderPanel* find_main_render_panel()
{
  const auto widgets = QApplication::allWidgets();
  for (QWidget* w : widgets) {
    if (auto rp = qobject_cast<rviz_common::RenderPanel*>(w)) return rp;
  }
  return nullptr;
}

// ---------- ctor/dtor ----------
RecordPanel::RecordPanel(QWidget* parent)
: rviz_common::Panel(parent)
{
  init_status_panel();
}

RecordPanel::~RecordPanel()
{
  stopRecordingInternal();
}

// ---------- RViz init ----------
void RecordPanel::onInitialize()
{
  auto ctx = getDisplayContext();
  if (!ctx) {
    RCLCPP_ERROR(getLogger(), "No RViz display context; panel init failed.");
    return;
  }
  auto ros_iface_weak = ctx->getRosNodeAbstraction();
  ros_iface_ = ros_iface_weak.lock();
  if (!ros_iface_) {
    RCLCPP_ERROR(getLogger(), "No RViz ROS node abstraction; panel init failed.");
    return;
  }
  ros_node_ = ros_iface_->get_raw_node();
  if (!ros_node_) {
    RCLCPP_ERROR(getLogger(), "No raw rclcpp node; panel init failed.");
    return;
  }

  render_panel_ = find_main_render_panel();
  if (!render_panel_) {
    RCLCPP_WARN(getLogger(), "Could not find RViz RenderPanel yet; will try to record anyway.");
  }

  using std::placeholders::_1; using std::placeholders::_2;
  start_srv_ = ros_node_->create_service<rviz_record::srv::StartRecording>(
      "/rviz_record/start", std::bind(&RecordPanel::startSrvCb, this, _1, _2));
  stop_srv_ = ros_node_->create_service<rviz_record::srv::StopRecording>(
      "/rviz_record/stop",  std::bind(&RecordPanel::stopSrvCb,  this, _1, _2));

  static std::once_flag gst_once;
  std::call_once(gst_once, [](){ gst_init(nullptr, nullptr); });

  RCLCPP_INFO(getLogger(), "rviz_record panel ready. Services: /rviz_record/start, /rviz_record/stop");
  update_status_panel();
}

rclcpp::Logger RecordPanel::getLogger() const
{
  return ros_node_ ? ros_node_->get_logger() : rclcpp::get_logger("rviz_record");
}

// ---------- status panel ----------
void RecordPanel::init_status_panel()
{
  auto* lay = new QVBoxLayout(this);
  lay->setContentsMargins(6, 6, 6, 6);
  status_label_ = new QLabel(this);
  status_label_->setWordWrap(true);
  status_label_->setStyleSheet(
    "QLabel { background: #222; color: #e0e0e0; border: 1px solid #444; "
    "border-radius: 4px; padding: 6px; font: 10pt 'DejaVu Sans Mono'; }");
  lay->addWidget(status_label_, 0, Qt::AlignTop);

  status_timer_ = new QTimer(this);
  status_timer_->setInterval(1000);
  connect(status_timer_, &QTimer::timeout, this, [this](){ update_status_panel(); });
  status_timer_->start();
}

void RecordPanel::update_status_panel()
{
  if (!status_label_) return;
  QString s;
  s += recording_.load() ? "● REC  " : "○ idle ";
  s += QString("fps=%1  scale=%2  frames=%3\n")
         .arg(fps_, 0, 'f', 1)
         .arg(scale_, 0, 'f', 2)
         .arg(static_cast<qulonglong>(frames_));
  s += QString("src=%1x%2  out=%3x%4  codec=%5\n")
         .arg(src_w_).arg(src_h_).arg(out_w_).arg(out_h_).arg(QString::fromStdString(codec_));
  s += QString("file: %1").arg(QString::fromStdString(filename_));
  status_label_->setText(s);
}

// ---------- services ----------
void RecordPanel::startSrvCb(
  const std::shared_ptr<rviz_record::srv::StartRecording::Request> req,
  std::shared_ptr<rviz_record::srv::StartRecording::Response> res)
{
  std::string err;
  if (startRecordingInternal(req->filename, req->fps, req->scale, req->codec, req->use_sim_time, err)) {
    res->message = "Recording started";
    res->accepted = true;
  } else {
    res->message = "Failed: " + err;
    res->accepted = false;
  }
  update_status_panel();
}

void RecordPanel::stopSrvCb(
  const std::shared_ptr<rviz_record::srv::StopRecording::Request> /*req*/,
  std::shared_ptr<rviz_record::srv::StopRecording::Response> res)
{
  auto stats = stopRecordingInternal();
  res->stopped      = true;
  res->output_path  = stats.output_path;
  res->frames       = stats.frames;
  res->duration_sec = stats.duration_sec;
  res->message      = stats.message;
  update_status_panel();
}

// ---------- UI grabber (GUI thread via QTimer) ----------
void RecordPanel::start_ui_grabber(int capture_fps)
{
  if (!render_panel_) render_panel_ = find_main_render_panel();
  if (!render_panel_) {
    RCLCPP_WARN(getLogger(), "No RenderPanel; UI grabber cannot start.");
    return;
  }

  // Prefer to have the RViz GL QWindow cached if your class exposes it
  if (!render_qwindow_) {
    if (auto* rw = render_panel_->getRenderWindow()) {
      render_qwindow_ = rw;  // rviz_rendering::RenderWindow*
    }
  }

  const int interval_ms = std::max(10, static_cast<int>(std::lround(1000.0 / std::max(1, capture_fps))));
  rviz_common::RenderPanel* w = render_panel_;
  

  QMetaObject::invokeMethod(w, [this, w, interval_ms]() {
    struct Grabber : QObject {
      rviz_common::RenderPanel* w; QTimer* t; std::function<void(const QImage&)> cb;
      Grabber(rviz_common::RenderPanel* w_, int interval, std::function<void(const QImage&)> cb_)
        : w(w_), t(new QTimer(this)), cb(std::move(cb_))
      {
        t->setTimerType(Qt::CoarseTimer);
        connect(t, &QTimer::timeout, this, [this]() {
          QImage img;
          if (!w || !w->isVisible()) return;

          // 1) Prefer capturing the RViz GL QWindow (just that surface)
          // rviz_rendering::RenderWindow* qwin =
          //     render_qwindow_ ? render_qwindow_ : w->getRenderWindow();
          rviz_rendering::RenderWindow* qwin = w->getRenderWindow();
          if (qwin) {
            QScreen* screen = qwin->screen();
            if (!screen) screen = QGuiApplication::primaryScreen();
            if (screen) {
              QPixmap pm = screen->grabWindow(qwin->winId());
              if (!pm.isNull()) {
                img = pm.toImage().convertToFormat(QImage::Format_RGBA8888);
              }
            }
          }

          if (img.isNull()) return;

          // Even dims for I420/x264
          int ew = (img.width()  & 1) ? img.width()  - 1 : img.width();
          int eh = (img.height() & 1) ? img.height() - 1 : img.height();
          if (ew != img.width() || eh != img.height()) img = img.copy(0, 0, ew, eh);

          if (img.width() >= 2 && img.height() >= 2) cb(img);
        });
        t->start(interval);
      }
    };

    auto store = [this](const QImage& img) {
      {
        std::lock_guard<std::mutex> lk(frame_mtx_);
        last_frame_ = img;
      }
      if (src_w_ == 0 || src_h_ == 0) {
        src_w_ = img.width();
        src_h_ = img.height();
        update_status_panel();
      }
    };
    grabber_ = new Grabber(w, interval_ms, store);
  }, Qt::QueuedConnection);
}

void RecordPanel::stop_ui_grabber()
{
  if (!grabber_) return;
  QObject* g = grabber_;
  grabber_.clear();
  QMetaObject::invokeMethod(g, [g]() {
    if (auto timer = g->findChild<QTimer*>()) timer->stop();
    g->deleteLater();
  }, Qt::QueuedConnection);
}

bool RecordPanel::get_latest_frame(QImage& out)
{
  std::lock_guard<std::mutex> lk(frame_mtx_);
  if (last_frame_.isNull() || last_frame_.width() < 2 || last_frame_.height() < 2) return false;
  out = last_frame_;
  return true;
}

// ---------- lifecycle ----------
bool RecordPanel::startRecordingInternal(const std::string& filename, double fps, double scale,
                                         const std::string& codec, bool use_sim_time, std::string& err_out)
{
  if (recording_.load()) { err_out = "Already recording"; return false; }
  if (filename.empty())  { err_out = "Filename is empty"; return false; }
  if (!(fps > 0.0 && fps <= 240.0)) { err_out = "fps must be (0,240]"; return false; }
  if (!(scale >= 0.1 && scale <= 4.0)) { err_out = "scale must be [0.1,4.0]"; return false; }

  std::string lc = codec;
  std::transform(lc.begin(), lc.end(), lc.begin(), ::tolower);
  if (lc != "h264" && lc != "vp9" && lc != "webm" && lc != "mkv") {
    err_out = "codec must be 'h264', 'vp9', or 'mkv/webm'";
    return false;
  }
  codec_ = (lc == "webm" ? "vp9" : lc); // alias

  filename_     = filename;
  fps_          = fps;
  scale_        = scale;
  use_sim_time_ = use_sim_time;

  frame_dur_ns_ = static_cast<int64_t>(std::llround(1e9 / fps_));
  ros_start_    = ros_clock_.now();
  pts0_ns_      = 0;
  last_pts_ns_  = 0;
  frames_       = 0;

  start_ui_grabber(std::max(30, static_cast<int>(std::round(fps))*2));

  stop_requested_.store(false);
  recording_.store(true);
  worker_ = std::thread(&RecordPanel::worker_loop, this);

  update_status_panel();
  RCLCPP_INFO(getLogger(), "Recording: %s (fps=%.1f scale=%.2f codec=%s sim_time=%s)",
              filename_.c_str(), fps_, scale_, codec_.c_str(), use_sim_time_ ? "true" : "false");
  return true;
}

RecordPanel::StopStats RecordPanel::stopRecordingInternal()
{
  StopStats s;
  s.output_path = filename_;
  s.frames = frames_;
  s.duration_sec = frames_ ? (frames_ / std::max(1.0, fps_)) : 0.0;
  s.message = "Stopped.";

  if (!recording_.load()) { s.message = "Not recording."; return s; }

  stop_requested_.store(true);
  stop_recording_blocking();
  if (worker_.joinable()) worker_.join();
  stop_ui_grabber();

  recording_.store(false);
  update_status_panel();
  return s;
}

// ---------- GStreamer ----------
bool RecordPanel::build_pipeline(const std::string& codec, const std::string& filename,
                                 int src_w, int src_h, int out_w, int out_h, int fps, std::string& err)
{
  pipeline_ = gst_pipeline_new("rvizrec");
  if (!pipeline_) { err = "gst_pipeline_new failed"; return false; }

  appsrc_                 = gst_element_factory_make("appsrc",       "src");
  GstElement* queue_in    = gst_element_factory_make("queue",        "queue_in");
  GstElement* convert     = gst_element_factory_make("videoconvert", "convert");
  GstElement* scale       = gst_element_factory_make("videoscale",   "scale");
  GstElement* rate        = gst_element_factory_make("videorate",    "rate");
  GstElement* capsfilter  = gst_element_factory_make("capsfilter",   "caps_out");
  GstElement* enc         = nullptr;
  GstElement* mux         = nullptr;
  GstElement* sink        = gst_element_factory_make("filesink",     "sink");

  if (!appsrc_ || !queue_in || !convert || !scale || !rate || !capsfilter || !sink) {
    err = "failed to create core elements";
    return false;
  }

  std::string c = codec;
  if (c == "mkv") c = "vp9";

  if (c == "h264") {
    enc = gst_element_factory_make("x264enc", "enc");
    mux = gst_element_factory_make("mp4mux",  "mux");
    if (!enc || !mux) { err = "failed to create x264enc/mp4mux"; return false; }
    g_object_set(enc, "speed-preset", 4, "key-int-max", fps * 2, NULL);
    g_object_set(mux, "faststart", TRUE, "streamable", TRUE, NULL);
  } else {
    enc = gst_element_factory_make("vp9enc",  "enc");
    const bool use_mkv = filename.size() >= 4 &&
                         (filename.rfind(".mkv") == filename.size() - 4 ||
                          filename.rfind(".MKV") == filename.size() - 4);
    mux = gst_element_factory_make(use_mkv ? "matroskamux" : "webmmux", "mux");
    if (!enc || !mux) { err = "failed to create vp9enc/(webm|mkv)mux"; return false; }
    g_object_set(enc, "deadline", 1, "crf", 33, NULL);
  }

  gst_bin_add_many(GST_BIN(pipeline_), appsrc_, queue_in, convert, scale, rate, capsfilter, enc, mux, sink, NULL);
  g_object_set(sink, "location", filename.c_str(), NULL);

  // appsrc caps (RGBA + size + fps)
  GstCaps* caps_in = gst_caps_new_simple(
      "video/x-raw",
      "format",     G_TYPE_STRING, "RGBA",
      "width",      G_TYPE_INT,    src_w,
      "height",     G_TYPE_INT,    src_h,
      "framerate",  GST_TYPE_FRACTION, fps, 1,
      NULL);
  if (!caps_in) { err = "caps_in alloc failed"; return false; }

  // output caps (I420 + fixed size + fps)
  GstCaps* caps_out = gst_caps_new_simple(
      "video/x-raw",
      "format",     G_TYPE_STRING, "I420",
      "width",      G_TYPE_INT,    out_w,
      "height",     G_TYPE_INT,    out_h,
      "framerate",  GST_TYPE_FRACTION, fps, 1,
      NULL);
  if (!caps_out) { gst_caps_unref(caps_in); err = "caps_out alloc failed"; return false; }

  g_object_set(G_OBJECT(appsrc_),
               "stream-type", 0,
               "format",      GST_FORMAT_TIME,
               "is-live",     TRUE,
               "do-timestamp", FALSE,
               NULL);
  gst_app_src_set_caps(GST_APP_SRC(appsrc_), caps_in);
  gst_caps_unref(caps_in);

  g_object_set(capsfilter, "caps", caps_out, NULL);
  gst_caps_unref(caps_out);

  if (!gst_element_link_many(appsrc_, queue_in, convert, scale, rate, capsfilter, enc, mux, sink, NULL)) {
    err = "element link failed";
    return false;
  }

  if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
    err = "failed to set pipeline PLAYING";
    return false;
  }
  return true;
}

bool RecordPanel::push_frame(const QImage& img, int64_t pts_ns, int64_t dur_ns)
{
  if (img.isNull()) return false;

  const int w = img.width();
  const int h = img.height();
  const int src_stride = img.bytesPerLine();
  const int dst_stride = w * 4; // RGBA tightly packed
  const size_t size = static_cast<size_t>(dst_stride) * static_cast<size_t>(h);

  GstBuffer* buffer = gst_buffer_new_allocate(nullptr, size, nullptr);
  if (!buffer) return false;

  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    gst_buffer_unref(buffer);
    return false;
  }

  const uint8_t* src = img.constBits();
  uint8_t* dst = map.data;
  if (src_stride == dst_stride) {
    memcpy(dst, src, size);
  } else {
    for (int y = 0; y < h; ++y) {
      memcpy(dst + static_cast<size_t>(y) * dst_stride,
             src + static_cast<size_t>(y) * src_stride,
             dst_stride);
    }
  }
  gst_buffer_unmap(buffer, &map);

  GST_BUFFER_PTS(buffer)      = pts_ns;
  GST_BUFFER_DTS(buffer)      = pts_ns;
  GST_BUFFER_DURATION(buffer) = dur_ns;

  return gst_app_src_push_buffer(GST_APP_SRC(appsrc_), buffer) == GST_FLOW_OK;
}

void RecordPanel::stop_recording_blocking()
{
  if (!pipeline_) return;

  if (appsrc_) {
    gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
    if (GstBus* bus = gst_element_get_bus(pipeline_)) {
      gst_bus_timed_pop_filtered(bus, 10 * GST_SECOND,
        static_cast<GstMessageType>(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
      gst_object_unref(bus);
    }
    gst_element_get_state(pipeline_, nullptr, nullptr, 5 * GST_SECOND);
  }

  teardown_pipeline();
}

void RecordPanel::teardown_pipeline()
{
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
  pipeline_ = nullptr;
  appsrc_   = nullptr; // owned by pipeline bin
}

// ---------- worker ----------
void RecordPanel::worker_loop()
{
  const int64_t period = frame_dur_ns_;
  auto wall_start = std::chrono::steady_clock::now();

  // Wait first frame
  for (;;) {
    if (stop_requested_.load()) return;
    QImage f; if (get_latest_frame(f)) break;
    std::this_thread::sleep_for(5ms);
  }

  while (!stop_requested_.load()) {
    int64_t target_pts_ns = 0;

    if (use_sim_time_) {
      auto now = ros_clock_.now();
      const int64_t delta_ns = (now - ros_start_).nanoseconds();
      if (delta_ns < last_pts_ns_ + period) { std::this_thread::sleep_for(2ms); continue; }
      target_pts_ns = delta_ns;
    } else {
      auto now = std::chrono::steady_clock::now();
      const int64_t elapsed_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now - wall_start).count();
      const int64_t next_pts   = (frames_ + 1) * period;
      if (elapsed_ns < next_pts) { std::this_thread::sleep_for(2ms); continue; }
      target_pts_ns = next_pts;
    }

    QImage frame;
    if (!get_latest_frame(frame)) { std::this_thread::sleep_for(2ms); continue; }

    if (!pipeline_) {
      src_w_ = frame.width();
      src_h_ = frame.height();
      int ow = static_cast<int>(std::lround(src_w_ * scale_));
      int oh = static_cast<int>(std::lround(src_h_ * scale_));
      out_w_ = make_even(ow);
      out_h_ = make_even(oh);

      std::string err;
      int fps_i = static_cast<int>(std::round(fps_));
      if (!build_pipeline(codec_, filename_, src_w_, src_h_, out_w_, out_h_, fps_i, err)) {
        RCLCPP_ERROR(getLogger(), "Pipeline build failed: %s", err.c_str());
        break;
      }
      update_status_panel();
    }

    if (!push_frame(frame, target_pts_ns, period)) {
      RCLCPP_ERROR(getLogger(), "appsrc push failed; stopping.");
      break;
    }

    frames_++;
    last_pts_ns_ = target_pts_ns;
  }

  if (appsrc_) gst_app_src_end_of_stream(GST_APP_SRC(appsrc_));
}

// ---------- plugin export ----------
} // namespace rviz_record

PLUGINLIB_EXPORT_CLASS(rviz_record::RecordPanel, rviz_common::Panel)
