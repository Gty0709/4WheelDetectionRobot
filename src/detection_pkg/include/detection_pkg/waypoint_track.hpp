#pragma once

#include <deque>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "detection_pkg/backprojection.hpp"

namespace detection_pkg
{

struct Observation
{
  double u{0.0};
  double v{0.0};
  double range_m{0.0};
  double map_x{0.0};
  double map_y{0.0};
  double confidence{0.0};
  std::string source;
};

class WaypointTrack
{
public:
  WaypointTrack(int buffer_size = 30, double uv_ema = 0.15, double map_ema = 1.0);

  bool matches_pixel(double u, double v, double tol_u, double tol_v) const;
  MapWaypoint fuse_map_1sigma(
    const Observation & obs, double sigma_floor_m,
    const std::optional<MapWaypoint> & prev = std::nullopt);

  std::optional<double> anchor_u;
  std::optional<double> anchor_v;

private:
  void update_anchor(const Observation & obs);

  int buffer_size_;
  double uv_ema_;
  double map_ema_;
  std::deque<Observation> observations_;
};

std::optional<Observation> observation_from_detection(
  double u, double v, double range_m, double confidence, const std::string & source,
  const cv::Mat & camera_matrix, const cv::Mat & t_map_camera);

}  // namespace detection_pkg
