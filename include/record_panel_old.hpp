#pragma once
#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <mutex>

#include <QWidget>
#include <QLabel>
#include <QImage>
#include <QPointer>

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include "rviz_record/srv/start_recording.hpp"
#include "rviz_record/srv/stop_recording.hpp"

// GStreamer
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>

namespace rviz_record
{

class RecordPanel : public rviz_common::Panel
{
  Q_OBJECT
public:
  explicit RecordPanel(QWidget* parent = nullptr) ;
  ~RecordPanel() override;

  void onInitialize() override;

private:
  // Service wiring
  void setup_services();
  void handle_start(
    const std::shared_ptr<rviz_record::srv::StartRecording::Request> req,
    std::shared_ptr<rviz_record::srv::StartRecording::Response> res);
  void handle_stop(
    const std::shared_ptr<rviz_record::srv::StopRecording::Request> /*req*/,
    std::shared_ptr<rviz_record::srv::StopRecording::Response> res);

  // Core control
  bool start_recording(
    const std::string& filename, double fps, double scale,
    const std::string& codec, bool use_sim_time, std::string& error_out);
  void stop_recording_blocking();
  void worker_loop();

  // Capture helpers
  rviz_common::RenderPanel* find_main_render_panel() const;
  QImage grab_frame_rgba();  // GUI thread via invokeMethod

  // GStreamer
  bool build_pipeline(const std::string& codec, const std::string& filename,
                      int src_w, int src_h, int out_w, int out_h, int fps, std::string& err);
  void teardown_pipeline();
  bool push_frame(const QImage& img, int64_t pts_ns, int64_t dur_ns);

private:
  // Frame sharing between UI grabber and encoder worker
  QImage last_frame_;
  std::mutex frame_mtx_;

  // UI-thread grabber object we create/destroy at start/stop
  QPointer<QObject> grabber_;  // lives in RViz (GUI) thread
  void start_ui_grabber(int capture_fps);
  void stop_ui_grabber();
  bool get_latest_frame(QImage &out);

private:
  // RViz / ROS
  rviz_common::RenderPanel* render_panel_{nullptr};
  rclcpp::Node::SharedPtr ros_node_;
  std::shared_ptr<rviz_common::ros_integration::RosNodeAbstractionIface> ros_iface_;

  // Services
  rclcpp::Service<rviz_record::srv::StartRecording>::SharedPtr srv_start_;
  rclcpp::Service<rviz_record::srv::StopRecording>::SharedPtr  srv_stop_;

  // Minimal UI
  QLabel* status_label_{nullptr};

  // State
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::thread worker_;
  std::mutex param_mtx_;

  // Frozen params per run
  std::string filename_;
  double fps_{30.0};
  double scale_{1.0};
  std::string codec_{"h264"};
  bool use_sim_time_{false};

  int src_w_{0}, src_h_{0};
  int out_w_{0}, out_h_{0};

  // GStreamer
  GstElement* pipeline_{nullptr};
  GstElement* appsrc_{nullptr};

  // Timing
  rclcpp::Clock ros_clock_{RCL_ROS_TIME};
  rclcpp::Time ros_start_;
  int64_t pts0_ns_{0};
  int64_t last_pts_ns_{0};
  int64_t frame_dur_ns_{0};
  uint64_t frames_{0};
};

} // namespace rviz_record
