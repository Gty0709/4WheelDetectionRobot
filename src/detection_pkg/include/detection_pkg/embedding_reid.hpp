#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include <opencv2/core.hpp>

#include "detection_pkg/trt_osnet.hpp"
#include "detection_pkg/waypoint_reid.hpp"

namespace detection_pkg
{

class WaypointReID
{
public:
  WaypointReID(
    const std::string & backend = "osnet", double cosine_min = 0.55,
    int orb_vocab_size = 48, double orb_match_score_min = 0.42);

  const std::string & active_backend() const;
  const std::string & fallback_reason() const;
  std::size_t size() const;
  std::pair<std::optional<int>, double> query(const cv::Mat & bgr);
  void register_waypoint(int waypoint_index, const cv::Mat & bgr);

private:
  std::string backend_;
  std::string active_;
  std::string fallback_reason_;
  std::unique_ptr<OsnetTrtEmbedding> osnet_;
  std::unique_ptr<OrbBowReID> orb_;
};

}  // namespace detection_pkg
