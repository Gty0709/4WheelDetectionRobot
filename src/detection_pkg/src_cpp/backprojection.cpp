#include "detection_pkg/backprojection.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <opencv2/calib3d.hpp>

namespace detection_pkg
{

namespace
{

cv::Mat mat_mul(const cv::Mat & a, const cv::Mat & b)
{
  return a * b;
}

cv::Mat mat_inv(const cv::Mat & m)
{
  return m.inv();
}

}  // namespace

double DetectionBox::center_u() const { return 0.5 * (x1 + x2); }
double DetectionBox::center_v() const { return 0.5 * (y1 + y2); }
double DetectionBox::area() const
{
  return std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
}

cv::Mat make_transform(const cv::Mat & rotation, const cv::Mat & translation)
{
  cv::Mat t = cv::Mat::eye(4, 4, CV_64F);
  rotation.copyTo(t(cv::Rect(0, 0, 3, 3)));
  t.at<double>(0, 3) = translation.at<double>(0, 0);
  t.at<double>(1, 3) = translation.at<double>(1, 0);
  t.at<double>(2, 3) = translation.at<double>(2, 0);
  return t;
}

cv::Mat camera_optical_fix()
{
  const double half_pi = 0.5 * CV_PI;
  const double cx = std::cos(-half_pi);
  const double sx = std::sin(-half_pi);
  const double cz = std::cos(-half_pi);
  const double sz = std::sin(-half_pi);

  cv::Mat rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cx, -sx, 0, sx, cx);
  cv::Mat rz = (cv::Mat_<double>(3, 3) << cz, -sz, 0, sz, cz, 0, 0, 0, 1);
  cv::Mat rot = rz * rx;
  cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);
  return make_transform(rot, zero);
}

cv::Mat camera_matrix_from_info(const std::array<double, 9> & k)
{
  return (cv::Mat_<double>(3, 3) <<
    k[0], k[1], k[2],
    k[3], k[4], k[5],
    k[6], k[7], k[8]);
}

cv::Vec3d pixel_to_ray_optical(double u, double v, const cv::Mat & camera_matrix)
{
  cv::Mat k_inv = mat_inv(camera_matrix);
  cv::Mat uv1 = (cv::Mat_<double>(3, 1) << u, v, 1.0);
  cv::Mat ray = k_inv * uv1;
  cv::Vec3d out(ray.at<double>(0), ray.at<double>(1), ray.at<double>(2));
  const double norm = cv::norm(out);
  if (norm > 1e-12) {
    out /= norm;
  }
  return out;
}

std::optional<cv::Vec3d> ray_ground_intersection(
  const cv::Vec3d & origin, const cv::Vec3d & direction, double z_ground)
{
  if (std::abs(direction[2]) < 1e-9) {
    return std::nullopt;
  }
  const double scale = (z_ground - origin[2]) / direction[2];
  if (scale <= 0.0) {
    return std::nullopt;
  }
  return origin + scale * direction;
}

cv::Vec3d camera_optical_origin(const cv::Mat & t_map_camera_link)
{
  const cv::Mat t_map_optical = mat_mul(t_map_camera_link, camera_optical_fix());
  return cv::Vec3d(
    t_map_optical.at<double>(0, 3),
    t_map_optical.at<double>(1, 3),
    t_map_optical.at<double>(2, 3));
}

double camera_range_in_map(const cv::Vec3d & point_map, const cv::Mat & t_map_camera_link)
{
  const cv::Vec3d origin = camera_optical_origin(t_map_camera_link);
  return cv::norm(point_map - origin);
}

double disparity_from_stereo(double u_l, double u_r) { return u_l - u_r; }

double disparity_at_depth(double fx, double baseline_m, double depth_m)
{
  if (depth_m <= 1e-6) {
    return 0.0;
  }
  return fx * baseline_m / depth_m;
}

double max_disparity_px(double fx, double baseline_m, double min_depth_m)
{
  return disparity_at_depth(fx, baseline_m, min_depth_m);
}

double min_disparity_for_max_depth(double fx, double baseline_m, double max_depth_m)
{
  return disparity_at_depth(fx, baseline_m, max_depth_m);
}

std::optional<double> validate_stereo_point(
  const cv::Vec3d & point_map, double u_l, double u_r, double fx, double baseline_m,
  const cv::Mat & t_map_camera_link, double min_depth_m, double max_depth_m,
  double max_ground_z_error, double min_disparity_px, double ground_z, double max_map_radius_m)
{
  if (!std::isfinite(point_map[0]) || !std::isfinite(point_map[1]) || !std::isfinite(point_map[2])) {
    return std::nullopt;
  }
  const double disparity = disparity_from_stereo(u_l, u_r);
  const double disp_max = max_disparity_px(fx, baseline_m, min_depth_m);
  const double disp_min = min_disparity_for_max_depth(fx, baseline_m, max_depth_m);
  if (disparity < min_disparity_px || disparity > disp_max || disparity < disp_min) {
    return std::nullopt;
  }
  const double range_m = camera_range_in_map(point_map, t_map_camera_link);
  if (range_m < min_depth_m || range_m > max_depth_m) {
    return std::nullopt;
  }
  if (std::abs(point_map[2] - ground_z) > max_ground_z_error) {
    return std::nullopt;
  }
  const double map_radius = std::hypot(point_map[0], point_map[1]);
  if (map_radius > max_map_radius_m) {
    return std::nullopt;
  }
  return range_m;
}

std::optional<double> validate_map_point(
  const cv::Vec3d & point_map, const cv::Mat & t_map_camera_link, double min_depth_m,
  double max_depth_m, double max_ground_z_error, double ground_z, double max_map_radius_m)
{
  if (!std::isfinite(point_map[0]) || !std::isfinite(point_map[1]) || !std::isfinite(point_map[2])) {
    return std::nullopt;
  }
  const double range_m = camera_range_in_map(point_map, t_map_camera_link);
  if (range_m < min_depth_m || range_m > max_depth_m) {
    return std::nullopt;
  }
  if (std::abs(point_map[2] - ground_z) > max_ground_z_error) {
    return std::nullopt;
  }
  const double map_radius = std::hypot(point_map[0], point_map[1]);
  if (map_radius > max_map_radius_m) {
    return std::nullopt;
  }
  return range_m;
}

std::optional<cv::Vec3d> triangulate_stereo(
  double u_l, double v_l, double u_r, double v_r, const cv::Mat & k_left,
  const cv::Mat & k_right, const cv::Mat & t_left_map, const cv::Mat & t_right_map)
{
  const auto projection_matrix = [](const cv::Mat & t_map_camlink, const cv::Mat & k) {
      const cv::Mat t_map_optical = mat_mul(t_map_camlink, camera_optical_fix());
      const cv::Mat t_optical_map = mat_inv(t_map_optical);
      cv::Mat r = t_optical_map(cv::Rect(0, 0, 3, 3)).clone();
      cv::Mat t = t_optical_map(cv::Rect(3, 0, 1, 3)).clone();
      cv::Mat rt;
      cv::hconcat(r, t, rt);
      return k * rt;
    };

  const cv::Mat p_left = projection_matrix(t_left_map, k_left);
  const cv::Mat p_right = projection_matrix(t_right_map, k_right);
  cv::Mat pts_left = (cv::Mat_<double>(2, 1) << u_l, v_l);
  cv::Mat pts_right = (cv::Mat_<double>(2, 1) << u_r, v_r);
  cv::Mat pts;
  cv::triangulatePoints(p_left, p_right, pts_left, pts_right, pts);
  if (std::abs(pts.at<double>(3, 0)) < 1e-9) {
    return std::nullopt;
  }
  const cv::Vec3d point(
    pts.at<double>(0, 0) / pts.at<double>(3, 0),
    pts.at<double>(1, 0) / pts.at<double>(3, 0),
    pts.at<double>(2, 0) / pts.at<double>(3, 0));
  if (!std::isfinite(point[0]) || !std::isfinite(point[1]) || !std::isfinite(point[2])) {
    return std::nullopt;
  }
  return point;
}

cv::Vec3d optical_point_to_map(const cv::Vec3d & point_optical, const cv::Mat & t_map_camera_link)
{
  const cv::Mat t_map_optical = mat_mul(t_map_camera_link, camera_optical_fix());
  cv::Mat p_h = (cv::Mat_<double>(4, 1) << point_optical[0], point_optical[1], point_optical[2], 1.0);
  cv::Mat out = t_map_optical * p_h;
  return cv::Vec3d(out.at<double>(0), out.at<double>(1), out.at<double>(2));
}

std::optional<std::tuple<double, double, std::string>> bbox_center_to_map(
  double u, double v, const cv::Mat & camera_matrix, const cv::Mat & t_map_camera_link,
  std::optional<double> depth_m, double ground_z)
{
  const cv::Vec3d ray_optical = pixel_to_ray_optical(u, v, camera_matrix);
  const cv::Mat t_map_optical = mat_mul(t_map_camera_link, camera_optical_fix());
  const cv::Vec3d origin(
    t_map_optical.at<double>(0, 3),
    t_map_optical.at<double>(1, 3),
    t_map_optical.at<double>(2, 3));
  cv::Mat rot = t_map_optical(cv::Rect(0, 0, 3, 3));
  cv::Mat dir_mat = rot * (cv::Mat_<double>(3, 1) << ray_optical[0], ray_optical[1], ray_optical[2]);
  const cv::Vec3d direction(dir_mat.at<double>(0), dir_mat.at<double>(1), dir_mat.at<double>(2));

  if (depth_m.has_value() && depth_m.value() > 0.05) {
    const cv::Vec3d point_optical = ray_optical * depth_m.value();
    const cv::Vec3d point_map = optical_point_to_map(point_optical, t_map_camera_link);
    return std::make_tuple(point_map[0], point_map[1], std::string("stereo"));
  }

  const auto hit = ray_ground_intersection(origin, direction, ground_z);
  if (!hit.has_value()) {
    return std::nullopt;
  }
  return std::make_tuple(hit->operator[](0), hit->operator[](1), std::string("ground"));
}

std::vector<std::pair<DetectionBox, DetectionBox>> match_stereo_detections(
  const std::vector<DetectionBox> & left_boxes, const std::vector<DetectionBox> & right_boxes,
  double row_tolerance_px, double area_ratio_min, double area_ratio_max,
  double min_disparity_px, double max_disparity_px)
{
  std::vector<DetectionBox> sorted_left = left_boxes;
  std::sort(sorted_left.begin(), sorted_left.end(), [](const DetectionBox & a, const DetectionBox & b) {
      return a.confidence > b.confidence;
    });

  std::vector<std::pair<DetectionBox, DetectionBox>> pairs;
  std::vector<bool> used_right(right_boxes.size(), false);

  for (const auto & left : sorted_left) {
    int best_idx = -1;
    double best_score = -1.0;
    for (std::size_t idx = 0; idx < right_boxes.size(); ++idx) {
      if (used_right[idx]) {
        continue;
      }
      const auto & right = right_boxes[idx];
      if (std::abs(left.center_v() - right.center_v()) > row_tolerance_px) {
        continue;
      }
      if (left.center_u() <= right.center_u()) {
        continue;
      }
      const double disparity = disparity_from_stereo(left.center_u(), right.center_u());
      if (disparity < min_disparity_px || disparity > max_disparity_px) {
        continue;
      }
      const double area_ratio = left.area() / std::max(right.area(), 1e-6);
      if (area_ratio < area_ratio_min || area_ratio > area_ratio_max) {
        continue;
      }
      const double row_penalty = std::abs(left.center_v() - right.center_v()) /
        std::max(row_tolerance_px, 1.0);
      const double score = left.confidence + right.confidence - 0.05 * row_penalty;
      if (score > best_score) {
        best_score = score;
        best_idx = static_cast<int>(idx);
      }
    }
    if (best_idx >= 0) {
      used_right[static_cast<std::size_t>(best_idx)] = true;
      pairs.emplace_back(left, right_boxes[static_cast<std::size_t>(best_idx)]);
    }
  }
  return pairs;
}

std::optional<double> stereo_depth_from_disparity(
  double u_l, double u_r, double fx, double baseline_m)
{
  const double disparity = u_l - u_r;
  if (disparity <= 0.5) {
    return std::nullopt;
  }
  return fx * baseline_m / disparity;
}

std::vector<MapWaypoint> deduplicate_waypoints(
  const std::vector<MapWaypoint> & waypoints, double radius_m)
{
  std::vector<MapWaypoint> sorted = waypoints;
  std::sort(sorted.begin(), sorted.end(), [](const MapWaypoint & a, const MapWaypoint & b) {
      return a.confidence > b.confidence;
    });
  std::vector<MapWaypoint> kept;
  for (const auto & wp : sorted) {
    bool far_enough = true;
    for (const auto & k : kept) {
      if (std::hypot(wp.x - k.x, wp.y - k.y) < radius_m) {
        far_enough = false;
        break;
      }
    }
    if (far_enough) {
      kept.push_back(wp);
    }
  }
  return kept;
}

std::vector<DetectionBox> nms_detection_boxes(
  const std::vector<DetectionBox> & boxes, double iou_threshold)
{
  std::vector<DetectionBox> sorted = boxes;
  std::sort(sorted.begin(), sorted.end(), [](const DetectionBox & a, const DetectionBox & b) {
      return a.confidence > b.confidence;
    });
  std::vector<DetectionBox> kept;
  const auto iou = [](const DetectionBox & a, const DetectionBox & b) {
      const double x1 = std::max(a.x1, b.x1);
      const double y1 = std::max(a.y1, b.y1);
      const double x2 = std::min(a.x2, b.x2);
      const double y2 = std::min(a.y2, b.y2);
      const double inter = std::max(0.0, x2 - x1) * std::max(0.0, y2 - y1);
      if (inter <= 0.0) {
        return 0.0;
      }
      const double denom = a.area() + b.area() - inter;
      return inter / std::max(denom, 1e-6);
    };
  for (const auto & box : sorted) {
    bool keep = true;
    for (const auto & k : kept) {
      if (iou(box, k) >= iou_threshold) {
        keep = false;
        break;
      }
    }
    if (keep) {
      kept.push_back(box);
    }
  }
  return kept;
}

}  // namespace detection_pkg
