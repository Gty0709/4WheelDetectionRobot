#pragma once

#include <atomic>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "detection_pkg/backprojection.hpp"
#include "detection_pkg/embedding_reid.hpp"
#include "detection_pkg/motion_compensation.hpp"
#include "detection_pkg/trt_inference.hpp"
#include "detection_pkg/waypoint_track.hpp"

namespace detection_pkg
{

class ClipDetectorNode : public rclcpp::Node
{
public:
  explicit ClipDetectorNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ClipDetectorNode() override;
  void shutdown_save();

private:
  void stop_ros_io();
  void on_left_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void on_right_info(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_cmd_vel_teleop(const geometry_msgs::msg::Twist::SharedPtr msg);
  void on_left_preview(const sensor_msgs::msg::Image::SharedPtr msg);
  void on_stereo_images(
    const sensor_msgs::msg::Image::ConstSharedPtr & left_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & right_msg);
  void on_finalize(const std_msgs::msg::Empty::SharedPtr msg);
  void periodic_waypoint_flush();
  void republish_markers();

  cv::Mat tf_to_matrix(const geometry_msgs::msg::Transform & tf) const;
  std::optional<cv::Mat> lookup_camera_tf(
    const builtin_interfaces::msg::Time & stamp, const std::string & camera_link);
  std::optional<cv::Mat> lookup_camera_tf_composed(
    const builtin_interfaces::msg::Time & stamp, const std::string & camera_link);
  std::tuple<std::optional<cv::Mat>, std::string, std::string, int, int, int>
  lookup_camera_tf_legacy(
    const builtin_interfaces::msg::Time & stamp, const std::string & camera_link);
  std::optional<cv::Mat> static_base_to_camera_tf(const std::string & camera_link);
  cv::Mat tf_matrix_at(
    const std::string & target, const std::string & source, const rclcpp::Time & query_time);
  std::pair<rclcpp::Time, int> sync_query_time_for(
    const std::string & target, const std::string & source,
    const builtin_interfaces::msg::Time & stamp);
  rclcpp::Time query_time_from_stamp(const builtin_interfaces::msg::Time & stamp);

  bool motion_allows_waypoint();
  void mark_fast_motion_if_needed();
  std::pair<double, double> effective_motion() const;
  void load_waypoints_from_disk();
  void health_check();
  std::vector<DetectionBox> nms_boxes(const std::vector<DetectionBox> & boxes);
  WaypointTrack new_track();
  WaypointTrack & ensure_track(std::size_t index);
  std::optional<std::size_t> find_track_by_pixel(double u, double v);
  std::pair<std::optional<std::size_t>, double> nearest_waypoint_index(double map_x, double map_y);
  bool apply_fused_track(std::size_t index, const Observation & obs);
  void add_waypoint(
    double u, double v, double range_m, double confidence, const std::string & source,
    const std::optional<cv::Mat> & crop_bgr, const cv::Mat & t_map_camera);
  std::optional<std::tuple<double, double, double, std::string>> ground_hit_map(
    double u, double v, const cv::Mat & t_map_camera);
  bool stereo_range_crosscheck(
    const DetectionBox & lb, const DetectionBox & rb, const cv::Mat & t_left,
    const cv::Mat & t_right, double ground_range_m);
  void process_stereo_detection(
    const DetectionBox & lb, const DetectionBox & rb, const cv::Mat & left_bgr,
    const cv::Mat & t_left, const cv::Mat & t_right);
  void process_mono_detection(
    const DetectionBox & lb, const cv::Mat & left_bgr, const cv::Mat & t_left);
  void publish_waypoints(bool live = true);
  void publish_markers(const std::vector<MapWaypoint> & wps);
  void publish_annotated(const cv::Mat & bgr, const std_msgs::msg::Header & header);
  cv::Mat draw_boxes(const cv::Mat & bgr, const std::vector<DetectionBox> & boxes);
  int write_waypoints_to_disk();
  void save_map_odom_offset();
  void render_waypoints_overlay();
  void finalize_waypoints(bool publish = true, bool force = false);
  void start_yolo_init_async(const std::string & weights);
  void join_yolo_init_thread();
  void detach_yolo_init_thread();

  // Parameters
  std::optional<std::filesystem::path> session_dir_;
  double conf_{0.5};
  double max_fps_{8.0};
  double ground_z_{0.002};
  double dedup_radius_{0.45};
  double save_dedup_radius_{0.65};
  double bearing_merge_deg_{6.0};
  double merge_max_blend_{0.2};
  double outlier_reject_m_{0.5};
  double max_angular_for_wp_{0.12};
  double min_linear_for_wp_{0.04};
  double max_linear_for_wp_{0.20};
  double motion_settle_sec_{0.4};
  bool enable_motion_gate_{false};
  bool use_odom_motion_gate_{false};
  double max_odom_angular_for_wp_{0.15};
  bool tf_prefer_image_stamp_{true};
  double stereo_sync_slop_{0.12};
  bool allow_ground_fallback_{true};
  double stereo_ground_crosscheck_m_{0.6};
  int max_tf_lag_ms_{250};
  bool enable_motion_compensation_{true};
  double waypoint_save_interval_{1.0};
  double marker_scale_{0.08};
  bool enable_reid_{true};
  int track_buffer_size_{30};
  double sigma_floor_m_{0.08};
  double pixel_merge_u_{48.0};
  double pixel_merge_v_{36.0};
  double track_uv_ema_{0.15};
  double track_map_ema_{0.3};
  double stereo_baseline_m_{0.0755};
  double min_depth_m_{0.6};
  double max_depth_m_{3.0};
  double min_disparity_px_{2.0};
  double max_ground_z_error_{0.15};
  double max_map_radius_m_{8.0};
  std::string map_frame_{"map"};
  std::string base_frame_{"base_footprint"};

  YoloInference yolo_;
  std::atomic<bool> yolo_ready_{false};
  std::atomic<bool> yolo_init_failed_{false};
  std::thread yolo_init_thread_;
  std::optional<WaypointReID> reid_;
  std::unique_ptr<OdomRingBuffer> odom_buffer_;
  std::map<std::string, cv::Mat> t_base_cam_cache_;

  cv::Mat k_left_;
  cv::Mat k_right_;
  bool have_k_left_{false};
  bool have_k_right_{false};
  std::vector<MapWaypoint> waypoints_;
  std::vector<WaypointTrack> tracks_;

  rclcpp::QoS camera_qos_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr wp_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr finalize_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_preview_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr left_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr right_info_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::TimerBase::SharedPtr flush_timer_;
  rclcpp::TimerBase::SharedPtr marker_timer_;

  using StereoSyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> left_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> right_sub_;
  std::shared_ptr<message_filters::Synchronizer<StereoSyncPolicy>> stereo_sync_;

  double last_infer_time_{0.0};
  double last_preview_time_{0.0};
  double last_waypoint_flush_time_{0.0};
  double last_fast_motion_mono_{0.0};
  double node_start_mono_{0.0};
  bool tf_ready_{false};
  bool warned_missing_info_{false};
  bool warned_tf_{false};
  bool warned_no_frames_{false};
  bool warned_yolo_loading_{false};
  bool warned_tf_fallback_{false};
  bool warned_tf_split_{false};
  bool finalize_overlay_done_{false};
  bool shutdown_done_{false};
  bool use_sim_time_{true};
  int stereo_frame_count_{0};
  int left_preview_count_{0};
  int marker_slot_count_{0};
  int last_pipeline_delay_ms_{0};
  int last_odom_extrapolate_ms_{0};
  int last_tf_lag_ms_{0};
  double cmd_linear_{0.0};
  double cmd_angular_{0.0};
  double teleop_linear_{0.0};
  double teleop_angular_{0.0};
  double odom_omega_{0.0};
  double odom_linear_{0.0};
  int motion_blocked_frames_{0};
  int stereo_reject_frames_{0};
  int merge_spatial_{0};
  int merge_ambiguous_skip_{0};
  int merge_new_{0};
  int merge_outlier_blocked_{0};
  int wp_obs_none_{0};
  int wp_depth_skip_{0};
  int tf_lag_high_{0};
};

}  // namespace detection_pkg
