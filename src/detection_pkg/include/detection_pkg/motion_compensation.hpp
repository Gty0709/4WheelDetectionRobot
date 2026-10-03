#pragma once

#include <deque>
#include <optional>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core.hpp>

namespace detection_pkg
{

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp);

struct OdomSample
{
  int64_t stamp_ns{0};
  cv::Vec3d position;
  cv::Mat rotation;  // 3x3 CV_64F
  double vx{0.0};
  double vy{0.0};
  double vz{0.0};
  double omega_z{0.0};

  cv::Mat to_matrix() const;
};

cv::Mat compose_map_camera_tf(
  const cv::Mat & t_map_odom, const cv::Mat & t_odom_base, const cv::Mat & t_base_camera);

OdomSample extrapolate_sample(const OdomSample & sample, double dt_sec);
OdomSample interpolate_samples(const OdomSample & s0, const OdomSample & s1, int64_t stamp_ns);

class OdomRingBuffer
{
public:
  OdomRingBuffer(
    double duration_sec = 2.0, int64_t interp_max_gap_ns = 80'000'000,
    int64_t max_extrapolate_ns = 120'000'000);

  std::size_t size() const;
  void add_from_odometry(const nav_msgs::msg::Odometry & msg);
  std::optional<cv::Mat> pose_at(int64_t stamp_ns);

  int last_extrapolate_ms{0};

private:
  void trim(int64_t newest_ns);

  int64_t duration_ns_;
  int64_t interp_max_gap_ns_;
  int64_t max_extrapolate_ns_;
  std::deque<OdomSample> samples_;
};

}  // namespace detection_pkg
