#include "detection_pkg/embedding_reid.hpp"

#include <algorithm>
#include <cctype>

namespace detection_pkg
{

WaypointReID::WaypointReID(
  const std::string & backend, double cosine_min, int orb_vocab_size, double orb_match_score_min)
: backend_(backend)
{
  std::string lowered = backend;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });

  if (lowered == "orb") {
    orb_ = std::make_unique<OrbBowReID>(
      std::max(orb_vocab_size, 8), orb_match_score_min);
    active_ = "orb";
    return;
  }

  osnet_ = std::make_unique<OsnetTrtEmbedding>(cosine_min);
  if (osnet_->available()) {
    active_ = "osnet";
  } else {
    fallback_reason_ = "osnet engine unavailable";
    osnet_.reset();
    orb_ = std::make_unique<OrbBowReID>(
      std::max(orb_vocab_size, 8), orb_match_score_min);
    active_ = "orb";
  }
}

const std::string & WaypointReID::active_backend() const { return active_; }
const std::string & WaypointReID::fallback_reason() const { return fallback_reason_; }

std::size_t WaypointReID::size() const
{
  if (active_ == "osnet" && osnet_) {
    return osnet_->size();
  }
  if (orb_) {
    return orb_->size();
  }
  return 0;
}

std::pair<std::optional<int>, double> WaypointReID::query(const cv::Mat & bgr)
{
  if (active_ == "osnet" && osnet_) {
    return osnet_->query(bgr);
  }
  if (orb_) {
    return orb_->query(bgr);
  }
  return {std::nullopt, 0.0};
}

void WaypointReID::register_waypoint(int waypoint_index, const cv::Mat & bgr)
{
  if (active_ == "osnet" && osnet_) {
    osnet_->register_waypoint(waypoint_index, bgr);
  } else if (orb_) {
    orb_->register_waypoint(waypoint_index, bgr);
  }
}

}  // namespace detection_pkg
