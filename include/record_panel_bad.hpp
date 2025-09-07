#pragma once

#include <rviz_common/panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <QImage>
#include <QPointer>
#include <QElapsedTimer>
#include <QLabel>

#include "rviz_record/srv/start_recording.hpp"
#include "rviz_record/srv/stop_recording.hpp"

namespace rviz_common { class RenderPanel; }
class QLabel;
class QTimer;
typedef struct _GstElement GstElement;

namespace rviz_rendering { class RenderWindow; }

namespace rviz_record
{

class RecordPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit RecordPanel(QWidget* parent = nullptr);
  ~RecordPanel() override;
  void onInitialize() override;

private:
  // ---- ROS / logging ----
  rclcpp::Logger getLogger() const;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> ros_iface_;
  rclcpp::Node::SharedPtr ros_node_;

  // Services
  void startSrvCb(
    const std::shared_ptr<rviz_record::srv::StartRecording::Request> req,
    std::shared_ptr<rviz_record::srv::StartRecording::Response> res);
  void stopSrvCb(
    const std::shared_ptr<rviz_record::srv::StopRecording::Request> req,
    std::shared_ptr<rviz_record::srv::StopRecording::Response> res);

  rclcpp::Service<rviz_record::srv::StartRecording>::SharedPtr start_srv_;
  rclcpp::Service<rviz_record::srv::StopRecording>::SharedPtr  stop_srv_;

  // ---- UI capture (runs on GUI thread via QTimer) ----
  void start_ui_grabber(int capture_fps);
  void stop_ui_grabber();
  bool get_latest_frame(QImage& out);

  QPointer<QObject> grabber_;   // owns the QTimer (lives in GUI thread)
  QImage            last_frame_;
  std::mutex        frame_mtx_;
  std::mutex        last_frame_mu_;


  rviz_common::RenderPanel* render_panel_ = nullptr;
  rviz_rendering::RenderWindow* render_qwindow_ = nullptr;

  // ---- Status (inside this plugin panel, not over the render view) ----
  void init_status_panel();
  void update_status_panel();
  QLabel* status_label_ = nullptr;
  QTimer* status_timer_ = nullptr;

  // ---- Recording lifecycle ----
  bool startRecordingInternal(const std::string& filename, double fps, double scale,
                              const std::string& codec, bool use_sim_time, std::string& err_out);

  struct StopStats {
    std::string output_path;
    uint64_t    frames = 0;
    double      duration_sec = 0.0;
    std::string message;
  };
  StopStats stopRecordingInternal();

  // worker thread (encoding)
  void worker_loop();

  // ---- GStreamer ----
  bool build_pipeline(const std::string& codec, const std::string& filename,
                      int src_w, int src_h, int out_w, int out_h, int fps, std::string& err);
  bool push_frame(const QImage& img, int64_t pts_ns, int64_t dur_ns);
  void stop_recording_blocking();
  void teardown_pipeline();

  GstElement* pipeline_ = nullptr;  // owns appsrc_
  GstElement* appsrc_   = nullptr;

  // ---- State & timing ----
  std::atomic<bool> recording_{false};
  std::atomic<bool> stop_requested_{false};
  std::thread       worker_;

  std::string filename_;
  double      fps_   = 10.0;
  double      scale_ = 1.0;
  std::string codec_ = "h264";
  bool        use_sim_time_ = false;

  int         src_w_ = 0, src_h_ = 0;
  int         out_w_ = 0, out_h_ = 0;

  int64_t     frame_dur_ns_ = 0;
  int64_t     pts0_ns_      = 0;
  int64_t     last_pts_ns_  = 0;
  uint64_t    frames_       = 0;

  rclcpp::Clock ros_clock_{RCL_ROS_TIME};
  rclcpp::Time  ros_start_{0, 0, RCL_ROS_TIME};
};

} // namespace rviz_record
