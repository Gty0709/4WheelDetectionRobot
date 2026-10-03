#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace detection_pkg
{

class OsnetTrtEmbedding
{
public:
  explicit OsnetTrtEmbedding(double cosine_min = 0.55, double ema_alpha = 0.3);
  ~OsnetTrtEmbedding();

  OsnetTrtEmbedding(const OsnetTrtEmbedding &) = delete;
  OsnetTrtEmbedding & operator=(const OsnetTrtEmbedding &) = delete;

  bool available() const;
  std::size_t size() const;
  std::pair<std::optional<int>, double> query(const cv::Mat & bgr);
  void register_waypoint(int waypoint_index, const cv::Mat & bgr);

  static std::string resolve_model_path();

#ifdef DETECTION_PKG_USE_TENSORRT
  struct TrtState;
#endif

private:
  std::optional<std::vector<float>> embed(const cv::Mat & bgr);
  static std::vector<float> normalize(const std::vector<float> & vec);
  static double cosine(const std::vector<float> & a, const std::vector<float> & b);

  double cosine_min_;
  double ema_alpha_;
  bool ready_{false};
  std::map<int, std::vector<float>> entries_;

#ifdef DETECTION_PKG_USE_TENSORRT
  std::unique_ptr<TrtState> trt_;
#endif
};

}  // namespace detection_pkg
