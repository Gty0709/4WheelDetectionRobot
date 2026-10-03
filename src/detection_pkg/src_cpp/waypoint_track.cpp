#include "detection_pkg/waypoint_track.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace detection_pkg
{

namespace
{

double spatial_median(std::vector<double> values)
{
  if (values.empty()) {
    return 0.0;
  }
  if (values.size() == 1) {
    return values.front();
  }
  const std::size_t mid = values.size() / 2;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  return values[mid];
}

std::vector<Observation> spatial_inliers(
  const std::vector<Observation> & observations, double sigma_floor_m)
{
  if (observations.size() <= 1) {
    return observations;
  }
  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(observations.size());
  ys.reserve(observations.size());
  for (const auto & o : observations) {
    xs.push_back(o.map_x);
    ys.push_back(o.map_y);
  }
  const double mx = spatial_median(xs);
  const double my = spatial_median(ys);
  std::vector<double> dists;
  dists.reserve(observations.size());
  for (const auto & o : observations) {
    dists.push_back(std::hypot(o.map_x - mx, o.map_y - my));
  }
  const double mu = std::accumulate(dists.begin(), dists.end(), 0.0) /
    static_cast<double>(dists.size());
  double sigma = 0.0;
  if (dists.size() > 1) {
    double variance = 0.0;
    for (const double d : dists) {
      variance += (d - mu) * (d - mu);
    }
    variance /= static_cast<double>(dists.size());
    sigma = std::sqrt(variance);
  }
  const double threshold = std::max(sigma, sigma_floor_m);
  std::vector<Observation> inliers;
  for (std::size_t i = 0; i < observations.size(); ++i) {
    if (dists[i] <= threshold) {
      inliers.push_back(observations[i]);
    }
  }
  return inliers.empty() ? observations : inliers;
}

}  // namespace

WaypointTrack::WaypointTrack(int buffer_size, double uv_ema, double map_ema)
: buffer_size_(buffer_size), uv_ema_(uv_ema), map_ema_(map_ema)
{
}

bool WaypointTrack::matches_pixel(double u, double v, double tol_u, double tol_v) const
{
  if (!anchor_u.has_value() || !anchor_v.has_value()) {
    return false;
  }
  return std::abs(u - anchor_u.value()) <= tol_u && std::abs(v - anchor_v.value()) <= tol_v;
}

void WaypointTrack::update_anchor(const Observation & obs)
{
  if (!anchor_u.has_value() || !anchor_v.has_value()) {
    anchor_u = obs.u;
    anchor_v = obs.v;
    return;
  }
  const double alpha = uv_ema_;
  anchor_u = (1.0 - alpha) * anchor_u.value() + alpha * obs.u;
  anchor_v = (1.0 - alpha) * anchor_v.value() + alpha * obs.v;
}

MapWaypoint WaypointTrack::fuse_map_1sigma(
  const Observation & obs, double sigma_floor_m, const std::optional<MapWaypoint> & prev)
{
  observations_.push_back(obs);
  while (static_cast<int>(observations_.size()) > buffer_size_) {
    observations_.pop_front();
  }
  update_anchor(obs);

  std::vector<Observation> obs_vec(observations_.begin(), observations_.end());
  const auto inliers = spatial_inliers(obs_vec, sigma_floor_m);

  std::vector<double> xs;
  std::vector<double> ys;
  xs.reserve(inliers.size());
  ys.reserve(inliers.size());
  double conf = 0.0;
  for (const auto & o : inliers) {
    xs.push_back(o.map_x);
    ys.push_back(o.map_y);
    conf = std::max(conf, o.confidence);
  }
  double mx = spatial_median(xs);
  double my = spatial_median(ys);
  std::string source = inliers.back().source;

  if (prev.has_value() && map_ema_ < 1.0) {
    const double alpha = map_ema_;
    mx = (1.0 - alpha) * prev->x + alpha * mx;
    my = (1.0 - alpha) * prev->y + alpha * my;
  }

  return MapWaypoint{mx, my, conf, source};
}

std::optional<Observation> observation_from_detection(
  double u, double v, double range_m, double confidence, const std::string & source,
  const cv::Mat & camera_matrix, const cv::Mat & t_map_camera)
{
  const auto mapped = bbox_center_to_map(u, v, camera_matrix, t_map_camera, range_m);
  if (!mapped.has_value()) {
    return std::nullopt;
  }
  Observation obs;
  obs.u = u;
  obs.v = v;
  obs.range_m = range_m;
  obs.map_x = std::get<0>(mapped.value());
  obs.map_y = std::get<1>(mapped.value());
  obs.confidence = confidence;
  obs.source = source;
  return obs;
}

}  // namespace detection_pkg
