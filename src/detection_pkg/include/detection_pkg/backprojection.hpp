#pragma once

#include <opencv2/core.hpp>

#include <array>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace detection_pkg
{

struct DetectionBox
{
  double x1{0.0};
  double y1{0.0};
  double x2{0.0};
  double y2{0.0};
  double confidence{0.0};
  int class_id{0};

  double center_u() const;
  double center_v() const;
  double area() const;
};

struct MapWaypoint
{
  double x{0.0};
  double y{0.0};
  double confidence{0.0};
  std::string source;
};

cv::Mat make_transform(const cv::Mat & rotation, const cv::Mat & translation);
cv::Mat camera_optical_fix();
cv::Mat camera_matrix_from_info(const std::array<double, 9> & k);
cv::Vec3d pixel_to_ray_optical(double u, double v, const cv::Mat & camera_matrix);
std::optional<cv::Vec3d> ray_ground_intersection(
  const cv::Vec3d & origin, const cv::Vec3d & direction, double z_ground);
cv::Vec3d camera_optical_origin(const cv::Mat & t_map_camera_link);
double camera_range_in_map(const cv::Vec3d & point_map, const cv::Mat & t_map_camera_link);
double disparity_from_stereo(double u_l, double u_r);
double disparity_at_depth(double fx, double baseline_m, double depth_m);
double max_disparity_px(double fx, double baseline_m, double min_depth_m);
double min_disparity_for_max_depth(double fx, double baseline_m, double max_depth_m);
std::optional<double> validate_stereo_point(
  const cv::Vec3d & point_map, double u_l, double u_r, double fx, double baseline_m,
  const cv::Mat & t_map_camera_link, double min_depth_m, double max_depth_m,
  double max_ground_z_error, double min_disparity_px, double ground_z = 0.002,
  double max_map_radius_m = 15.0);
std::optional<double> validate_map_point(
  const cv::Vec3d & point_map, const cv::Mat & t_map_camera_link, double min_depth_m,
  double max_depth_m, double max_ground_z_error, double ground_z = 0.002,
  double max_map_radius_m = 15.0);
std::optional<cv::Vec3d> triangulate_stereo(
  double u_l, double v_l, double u_r, double v_r, const cv::Mat & k_left,
  const cv::Mat & k_right, const cv::Mat & t_left_map, const cv::Mat & t_right_map);
cv::Vec3d optical_point_to_map(const cv::Vec3d & point_optical, const cv::Mat & t_map_camera_link);
std::optional<std::tuple<double, double, std::string>> bbox_center_to_map(
  double u, double v, const cv::Mat & camera_matrix, const cv::Mat & t_map_camera_link,
  std::optional<double> depth_m = std::nullopt, double ground_z = 0.002);
std::vector<std::pair<DetectionBox, DetectionBox>> match_stereo_detections(
  const std::vector<DetectionBox> & left_boxes, const std::vector<DetectionBox> & right_boxes,
  double row_tolerance_px = 8.0, double area_ratio_min = 0.5, double area_ratio_max = 2.0,
  double min_disparity_px = 2.0, double max_disparity_px = 1e6);
std::optional<double> stereo_depth_from_disparity(
  double u_l, double u_r, double fx, double baseline_m);
std::vector<MapWaypoint> deduplicate_waypoints(
  const std::vector<MapWaypoint> & waypoints, double radius_m = 0.12);
std::vector<DetectionBox> nms_detection_boxes(
  const std::vector<DetectionBox> & boxes, double iou_threshold = 0.45);

}  // namespace detection_pkg
