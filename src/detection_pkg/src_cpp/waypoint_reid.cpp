#include "detection_pkg/waypoint_reid.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace detection_pkg
{

namespace
{
constexpr int kOrbDim = 32;
}

cv::Mat crop_detection_patch(
  const cv::Mat & bgr, double x1, double y1, double x2, double y2,
  double pad_ratio, int min_side)
{
  if (bgr.empty()) {
    return {};
  }
  const int h = bgr.rows;
  const int w = bgr.cols;
  const double bw = std::max(1.0, x2 - x1);
  const double bh = std::max(1.0, y2 - y1);
  const double px = pad_ratio * bw;
  const double py = pad_ratio * bh;
  const int ix1 = std::max(0, static_cast<int>(std::floor(x1 - px)));
  const int iy1 = std::max(0, static_cast<int>(std::floor(y1 - py)));
  const int ix2 = std::min(w, static_cast<int>(std::ceil(x2 + px)));
  const int iy2 = std::min(h, static_cast<int>(std::ceil(y2 + py)));
  if (ix2 - ix1 < min_side || iy2 - iy1 < min_side) {
    return {};
  }
  return bgr(cv::Rect(ix1, iy1, ix2 - ix1, iy2 - iy1)).clone();
}

OrbBowReID::OrbBowReID(
  int vocab_size, double match_score_min, int orb_features, int min_descriptors, int vocab_warmup)
: vocab_size_(std::max(8, vocab_size)),
  match_score_min_(match_score_min),
  min_descriptors_(min_descriptors),
  vocab_warmup_(vocab_warmup),
  orb_(cv::ORB::create(orb_features))
{
}

std::size_t OrbBowReID::size() const { return entries_.size(); }

int OrbBowReID::bow_dim() const
{
  return vocab_.empty() ? kOrbDim : vocab_.rows;
}

cv::Mat OrbBowReID::empty_bow() const
{
  return cv::Mat::zeros(bow_dim(), 1, CV_32F);
}

cv::Mat OrbBowReID::extract_descriptors(const cv::Mat & bgr)
{
  if (bgr.empty()) {
    return {};
  }
  cv::Mat gray;
  cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
  std::vector<cv::Mat> chunks;
  for (const double scale : {1.0, 2.0}) {
    cv::Mat g = gray;
    if (scale != 1.0) {
      cv::resize(gray, g, cv::Size(), scale, scale);
    }
    std::vector<cv::KeyPoint> kp;
    cv::Mat des;
    orb_->detectAndCompute(g, cv::noArray(), kp, des);
    if (!des.empty()) {
      chunks.push_back(des);
    }
  }
  if (chunks.empty()) {
    return {};
  }
  cv::Mat merged;
  cv::vconcat(chunks, merged);
  if (merged.rows < min_descriptors_) {
    return {};
  }
  if (merged.type() != CV_8U) {
    merged.convertTo(merged, CV_8U);
  }
  return merged;
}

cv::Mat OrbBowReID::descriptors_for_kmeans(const cv::Mat & des)
{
  cv::Mat out;
  des.convertTo(out, CV_32F);
  return out;
}

cv::Mat OrbBowReID::color_histogram(const cv::Mat & bgr)
{
  cv::Mat hsv;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  const int hist_size[] = {16, 4, 4};
  const float hranges[] = {0, 180};
  const float sranges[] = {0, 256};
  const float vranges[] = {0, 256};
  const float * ranges[] = {hranges, sranges, vranges};
  const int channels[] = {0, 1, 2};
  cv::Mat hist;
  cv::calcHist(&hsv, 1, channels, cv::noArray(), hist, 3, hist_size, ranges, true, false);
  hist.convertTo(hist, CV_32F);
  const double s = cv::sum(hist)[0];
  if (s > 1e-6) {
    hist /= static_cast<float>(s);
  }
  return hist.reshape(1, 1);
}

void OrbBowReID::build_vocab()
{
  if (!vocab_.empty() || static_cast<int>(pending_descriptors_.size()) < vocab_warmup_) {
    return;
  }
  std::vector<cv::Mat> converted;
  converted.reserve(pending_descriptors_.size());
  for (const auto & d : pending_descriptors_) {
    converted.push_back(descriptors_for_kmeans(d));
  }
  cv::Mat stacked;
  cv::vconcat(converted, stacked);
  const int k = std::min(vocab_size_, stacked.rows);
  if (k < 4) {
    return;
  }
  cv::Mat labels;
  cv::Mat centers;
  const cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 12, 0.5);
  cv::kmeans(stacked, k, labels, criteria, 3, cv::KMEANS_PP_CENTERS, centers);
  vocab_ = centers;
  for (auto & [idx, entry] : entries_) {
    (void)idx;
    if (!entry.descriptors.empty()) {
      entry.bow = descriptors_to_bow(entry.descriptors);
    } else {
      entry.bow = empty_bow();
    }
  }
}

cv::Mat OrbBowReID::descriptors_to_bow(const cv::Mat & des)
{
  cv::Mat des_f = descriptors_for_kmeans(des);
  if (vocab_.empty()) {
    cv::Mat vec = des_f.reshape(1, 1);
    cv::reduce(vec, vec, 0, cv::REDUCE_AVG);
    const double norm = cv::norm(vec);
    if (norm > 1e-6) {
      vec /= norm;
    }
    return vec.t();
  }
  cv::Mat diff;
  for (int i = 0; i < des_f.rows; ++i) {
    cv::Mat row = des_f.row(i);
    cv::Mat distances;
    for (int j = 0; j < vocab_.rows; ++j) {
      cv::Mat d = row - vocab_.row(j);
      distances.push_back(cv::norm(d));
    }
    double min_val{0.0};
    cv::Point min_loc;
    cv::minMaxLoc(distances.reshape(1, 1), &min_val, nullptr, &min_loc);
    diff.push_back(static_cast<float>(min_loc.x));
  }
  cv::Mat hist = cv::Mat::zeros(vocab_.rows, 1, CV_32F);
  for (int i = 0; i < diff.rows; ++i) {
    const int label = static_cast<int>(diff.at<float>(i, 0));
    if (label >= 0 && label < hist.rows) {
      hist.at<float>(label, 0) += 1.0f;
    }
  }
  const double s = cv::sum(hist)[0];
  if (s > 1e-6) {
    hist /= static_cast<float>(s);
  }
  return hist;
}

double OrbBowReID::histogram_intersection(const cv::Mat & a, const cv::Mat & b)
{
  const int n = std::min(a.rows * a.cols, b.rows * b.cols);
  if (n == 0) {
    return 0.0;
  }
  cv::Mat a_flat = a.reshape(1, 1);
  cv::Mat b_flat = b.reshape(1, 1);
  double sum = 0.0;
  for (int i = 0; i < n; ++i) {
    sum += std::min(a_flat.at<float>(0, i), b_flat.at<float>(0, i));
  }
  return sum;
}

double OrbBowReID::descriptor_match_score(const cv::Mat & des, const ReIDEntry & entry) const
{
  if (entry.descriptors.empty() || entry.descriptors.rows < 4) {
    return 0.0;
  }
  cv::Mat q = des;
  cv::Mat t = entry.descriptors;
  if (q.type() != CV_8U) {
    q.convertTo(q, CV_8U);
  }
  if (t.type() != CV_8U) {
    t.convertTo(t, CV_8U);
  }
  cv::BFMatcher matcher(cv::NORM_HAMMING, false);
  std::vector<std::vector<cv::DMatch>> pairs;
  try {
    matcher.knnMatch(q, t, pairs, 2);
  } catch (const cv::Exception &) {
    return 0.0;
  }
  int good = 0;
  for (const auto & pair : pairs) {
    if (pair.size() < 2) {
      continue;
    }
    if (pair[0].distance < 0.75 * pair[1].distance) {
      ++good;
    }
  }
  return static_cast<double>(good) / std::max(q.rows, 1);
}

std::pair<std::optional<int>, double> OrbBowReID::query(const cv::Mat & bgr)
{
  if (entries_.empty()) {
    return {std::nullopt, 0.0};
  }
  const cv::Mat des = extract_descriptors(bgr);
  const cv::Mat color = color_histogram(bgr);
  cv::Mat bow;
  if (!des.empty()) {
    bow = descriptors_to_bow(des);
  }
  std::optional<int> best_idx;
  double best_score = 0.0;
  for (const auto & [idx, entry] : entries_) {
    const double color_score = histogram_intersection(color, entry.color_hist);
    double score = color_score;
    if (!bow.empty() && !entry.descriptors.empty()) {
      const double bow_score = histogram_intersection(bow, entry.bow);
      const double des_score = descriptor_match_score(des, entry);
      score = 0.40 * bow_score + 0.35 * std::min(des_score, 1.0) + 0.25 * color_score;
    }
    if (score > best_score) {
      best_score = score;
      best_idx = idx;
    }
  }
  if (best_idx.has_value() && best_score >= match_score_min_) {
    return {best_idx, best_score};
  }
  return {std::nullopt, best_score};
}

void OrbBowReID::register_waypoint(int waypoint_index, const cv::Mat & bgr)
{
  cv::Mat des = extract_descriptors(bgr);
  const cv::Mat color = color_histogram(bgr);
  cv::Mat bow;
  if (!des.empty()) {
    pending_descriptors_.push_back(des);
    build_vocab();
    bow = descriptors_to_bow(des);
  } else if (entries_.count(waypoint_index) > 0) {
    auto & entry = entries_.at(waypoint_index);
    entry.color_hist = 0.7 * entry.color_hist + 0.3 * color;
    const double s = cv::sum(entry.color_hist)[0];
    if (s > 1e-6) {
      entry.color_hist /= static_cast<float>(s);
    }
    return;
  } else {
    bow = empty_bow();
    des = cv::Mat();
  }

  if (entries_.count(waypoint_index) > 0) {
    auto & entry = entries_.at(waypoint_index);
    entry.color_hist = 0.7 * entry.color_hist + 0.3 * color;
    double s = cv::sum(entry.color_hist)[0];
    if (s > 1e-6) {
      entry.color_hist /= static_cast<float>(s);
    }
    if (!des.empty()) {
      cv::Mat new_bow = descriptors_to_bow(des);
      if (entry.bow.rows != new_bow.rows) {
        entry.bow = new_bow;
      } else {
        entry.bow = 0.7 * entry.bow + 0.3 * new_bow;
        s = cv::sum(entry.bow)[0];
        if (s > 1e-6) {
          entry.bow /= static_cast<float>(s);
        }
      }
      entry.descriptors = des;
    }
  } else {
    entries_[waypoint_index] = ReIDEntry{bow, color, des};
  }
}

}  // namespace detection_pkg
