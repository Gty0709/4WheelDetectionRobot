#pragma once

#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "detection_pkg/backprojection.hpp"

namespace detection_pkg
{

enum class YoloBackend
{
  TensorRT,
  OpenCVDnn,
  None
};

struct LetterboxInfo
{
  float scale{1.0f};
  int pad_x{0};
  int pad_y{0};
  int orig_w{0};
  int orig_h{0};
};

class YoloInference
{
public:
  explicit YoloInference(double conf_threshold = 0.5, int input_size = 640);
  ~YoloInference();

  YoloInference(const YoloInference &) = delete;
  YoloInference & operator=(const YoloInference &) = delete;

  bool init(const std::string & weights_param = "");
  void set_conf_threshold(double conf) { conf_threshold_ = conf; }
  YoloBackend backend() const;
  std::vector<DetectionBox> detect(const cv::Mat & bgr);

  static std::string resolve_model_paths(
    std::string & engine_path, std::string & onnx_path);

#ifdef DETECTION_PKG_USE_TENSORRT
  struct TrtState;
#endif

private:
  cv::Mat preprocess(const cv::Mat & bgr, LetterboxInfo & info) const;
  std::vector<DetectionBox> postprocess(
    const float * output, int num_detections, int det_stride,
    const LetterboxInfo & info) const;
  DetectionBox map_box_to_original(
    double x1, double y1, double x2, double y2, double conf, int cls,
    const LetterboxInfo & info) const;

  double conf_threshold_;
  int input_size_;
  YoloBackend backend_{YoloBackend::None};
  std::string engine_path_;
  std::string onnx_path_;

#ifdef DETECTION_PKG_USE_TENSORRT
  std::unique_ptr<TrtState> trt_;
#endif
  cv::dnn::Net dnn_net_;
  bool dnn_ready_{false};
};

}  // namespace detection_pkg
