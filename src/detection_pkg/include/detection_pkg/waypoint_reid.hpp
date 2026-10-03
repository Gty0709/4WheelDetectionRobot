#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>

namespace detection_pkg
{

cv::Mat crop_detection_patch(
  const cv::Mat & bgr, double x1, double y1, double x2, double y2,
  double pad_ratio = 0.12, int min_side = 24);

struct ReIDEntry
{
  cv::Mat bow;
  cv::Mat color_hist;
  cv::Mat descriptors;
};

class OrbBowReID
{
public:
  OrbBowReID(
    int vocab_size = 48, double match_score_min = 0.42, int orb_features = 200,
    int min_descriptors = 6, int vocab_warmup = 4);

  std::size_t size() const;
  std::pair<std::optional<int>, double> query(const cv::Mat & bgr);
  void register_waypoint(int waypoint_index, const cv::Mat & bgr);

private:
  int bow_dim() const;
  cv::Mat empty_bow() const;
  cv::Mat extract_descriptors(const cv::Mat & bgr);
  static cv::Mat descriptors_for_kmeans(const cv::Mat & des);
  static cv::Mat color_histogram(const cv::Mat & bgr);
  void build_vocab();
  cv::Mat descriptors_to_bow(const cv::Mat & des);
  static double histogram_intersection(const cv::Mat & a, const cv::Mat & b);
  double descriptor_match_score(const cv::Mat & des, const ReIDEntry & entry) const;

  int vocab_size_;
  double match_score_min_;
  int min_descriptors_;
  int vocab_warmup_;
  cv::Ptr<cv::ORB> orb_;
  cv::Mat vocab_;
  std::map<int, ReIDEntry> entries_;
  std::vector<cv::Mat> pending_descriptors_;
};

}  // namespace detection_pkg
