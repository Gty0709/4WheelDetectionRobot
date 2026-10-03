#include "detection_pkg/motion_compensation.hpp"

#include "detection_pkg/backprojection.hpp"

#include <algorithm>
#include <cmath>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

namespace detection_pkg
{

namespace
{

cv::Mat rotation_from_yaw(double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return (cv::Mat_<double>(3, 3) << c, -s, 0, s, c, 0, 0, 0, 1);
}

double yaw_from_rotation(const cv::Mat & rot)
{
  tf2::Matrix3x3 m(
    rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
    rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
    rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

cv::Mat slerp_rotation(const cv::Mat & r0, const cv::Mat & r1, double alpha)
{
  tf2::Matrix3x3 m0(
    r0.at<double>(0, 0), r0.at<double>(0, 1), r0.at<double>(0, 2),
    r0.at<double>(1, 0), r0.at<double>(1, 1), r0.at<double>(1, 2),
    r0.at<double>(2, 0), r0.at<double>(2, 1), r0.at<double>(2, 2));
  tf2::Matrix3x3 m1(
    r1.at<double>(0, 0), r1.at<double>(0, 1), r1.at<double>(0, 2),
    r1.at<double>(1, 0), r1.at<double>(1, 1), r1.at<double>(1, 2),
    r1.at<double>(2, 0), r1.at<double>(2, 1), r1.at<double>(2, 2));
  tf2::Quaternion q0;
  tf2::Quaternion q1;
  m0.getRotation(q0);
  m1.getRotation(q1);
  const tf2::Quaternion q_interp = q0.slerp(q1, alpha);
  tf2::Matrix3x3 m_interp(q_interp);
  return (cv::Mat_<double>(3, 3) <<
    m_interp[0][0], m_interp[0][1], m_interp[0][2],
    m_interp[1][0], m_interp[1][1], m_interp[1][2],
    m_interp[2][0], m_interp[2][1], m_interp[2][2]);
}

}  // namespace

int64_t stamp_to_ns(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000LL + static_cast<int64_t>(stamp.nanosec);
}

cv::Mat OdomSample::to_matrix() const
{
  return make_transform(rotation, (cv::Mat_<double>(3, 1) << position[0], position[1], position[2]));
}

cv::Mat compose_map_camera_tf(
  const cv::Mat & t_map_odom, const cv::Mat & t_odom_base, const cv::Mat & t_base_camera)
{
  return t_map_odom * t_odom_base * t_base_camera;
}

OdomSample extrapolate_sample(const OdomSample & sample, double dt_sec)
{
  const double yaw = yaw_from_rotation(sample.rotation);
  const double cos_y = std::cos(yaw);
  const double sin_y = std::sin(yaw);
  const double dx = (cos_y * sample.vx - sin_y * sample.vy) * dt_sec;
  const double dy = (sin_y * sample.vx + cos_y * sample.vy) * dt_sec;
  const double dz = sample.vz * dt_sec;
  const double yaw_new = yaw + sample.omega_z * dt_sec;
  OdomSample out = sample;
  out.stamp_ns = sample.stamp_ns + static_cast<int64_t>(dt_sec * 1e9);
  out.position = sample.position + cv::Vec3d(dx, dy, dz);
  out.rotation = rotation_from_yaw(yaw_new);
  return out;
}

OdomSample interpolate_samples(const OdomSample & s0, const OdomSample & s1, int64_t stamp_ns)
{
  if (s1.stamp_ns == s0.stamp_ns) {
    return s0;
  }
  double alpha = static_cast<double>(stamp_ns - s0.stamp_ns) /
    static_cast<double>(s1.stamp_ns - s0.stamp_ns);
  alpha = std::min(std::max(alpha, 0.0), 1.0);
  OdomSample out;
  out.stamp_ns = stamp_ns;
  out.position = (1.0 - alpha) * s0.position + alpha * s1.position;
  out.rotation = slerp_rotation(s0.rotation, s1.rotation, alpha);
  out.vx = (1.0 - alpha) * s0.vx + alpha * s1.vx;
  out.vy = (1.0 - alpha) * s0.vy + alpha * s1.vy;
  out.vz = (1.0 - alpha) * s0.vz + alpha * s1.vz;
  out.omega_z = (1.0 - alpha) * s0.omega_z + alpha * s1.omega_z;
  return out;
}

OdomRingBuffer::OdomRingBuffer(
  double duration_sec, int64_t interp_max_gap_ns, int64_t max_extrapolate_ns)
: duration_ns_(std::max(static_cast<int64_t>(duration_sec * 1e9), static_cast<int64_t>(1))),
  interp_max_gap_ns_(std::max(interp_max_gap_ns, static_cast<int64_t>(1))),
  max_extrapolate_ns_(std::max(max_extrapolate_ns, static_cast<int64_t>(0)))
{
}

std::size_t OdomRingBuffer::size() const { return samples_.size(); }

void OdomRingBuffer::add_from_odometry(const nav_msgs::msg::Odometry & msg)
{
  const auto & pos = msg.pose.pose.position;
  const auto & ori = msg.pose.pose.orientation;
  const auto & twist = msg.twist.twist;
  tf2::Quaternion q(ori.x, ori.y, ori.z, ori.w);
  tf2::Matrix3x3 m(q);
  OdomSample sample;
  sample.stamp_ns = stamp_to_ns(msg.header.stamp);
  sample.position = cv::Vec3d(pos.x, pos.y, pos.z);
  sample.rotation = (cv::Mat_<double>(3, 3) <<
    m[0][0], m[0][1], m[0][2],
    m[1][0], m[1][1], m[1][2],
    m[2][0], m[2][1], m[2][2]);
  sample.vx = twist.linear.x;
  sample.vy = twist.linear.y;
  sample.vz = twist.linear.z;
  sample.omega_z = twist.angular.z;
  samples_.push_back(sample);
  trim(sample.stamp_ns);
}

void OdomRingBuffer::trim(int64_t newest_ns)
{
  const int64_t cutoff = newest_ns - duration_ns_;
  while (!samples_.empty() && samples_.front().stamp_ns < cutoff) {
    samples_.pop_front();
  }
}

std::optional<cv::Mat> OdomRingBuffer::pose_at(int64_t stamp_ns)
{
  last_extrapolate_ms = 0;
  if (samples_.empty()) {
    return std::nullopt;
  }

  if (stamp_ns < samples_.front().stamp_ns) {
    const int64_t gap = samples_.front().stamp_ns - stamp_ns;
    if (gap > max_extrapolate_ns_) {
      return std::nullopt;
    }
    const double dt_sec = -static_cast<double>(gap) / 1e9;
    const OdomSample sample = extrapolate_sample(samples_.front(), dt_sec);
    last_extrapolate_ms = static_cast<int>(gap / 1'000'000);
    return sample.to_matrix();
  }

  if (stamp_ns > samples_.back().stamp_ns) {
    const int64_t gap = stamp_ns - samples_.back().stamp_ns;
    if (gap > max_extrapolate_ns_) {
      return std::nullopt;
    }
    const double dt_sec = static_cast<double>(gap) / 1e9;
    const OdomSample sample = extrapolate_sample(samples_.back(), dt_sec);
    last_extrapolate_ms = static_cast<int>(gap / 1'000'000);
    return sample.to_matrix();
  }

  std::vector<int64_t> stamps;
  stamps.reserve(samples_.size());
  for (const auto & s : samples_) {
    stamps.push_back(s.stamp_ns);
  }
  const auto it = std::lower_bound(stamps.begin(), stamps.end(), stamp_ns);
  const std::size_t idx = static_cast<std::size_t>(it - stamps.begin());
  if (idx == 0) {
    return samples_.front().to_matrix();
  }
  if (idx >= samples_.size()) {
    return samples_.back().to_matrix();
  }
  const OdomSample & s0 = samples_[idx - 1];
  const OdomSample & s1 = samples_[idx];
  if (s1.stamp_ns - s0.stamp_ns > interp_max_gap_ns_) {
    return std::nullopt;
  }
  return interpolate_samples(s0, s1, stamp_ns).to_matrix();
}

}  // namespace detection_pkg
