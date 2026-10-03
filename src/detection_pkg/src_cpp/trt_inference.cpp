#include "detection_pkg/trt_inference.hpp"

#include <cstring>
#include <fstream>
#include <iostream>
#include <filesystem>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#ifdef DETECTION_PKG_USE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace detection_pkg
{

#ifdef DETECTION_PKG_USE_TENSORRT
struct YoloInference::TrtState
{
  nvinfer1::IRuntime * runtime{nullptr};
  nvinfer1::ICudaEngine * engine{nullptr};
  nvinfer1::IExecutionContext * context{nullptr};
  cudaStream_t stream{};
  void * device_input{nullptr};
  void * device_output{nullptr};
  std::size_t input_bytes{0};
  std::size_t output_bytes{0};
  int num_detections{300};
  int det_stride{6};

  ~TrtState()
  {
    if (device_input) {
      cudaFree(device_input);
    }
    if (device_output) {
      cudaFree(device_output);
    }
    if (stream) {
      cudaStreamDestroy(stream);
    }
    delete context;
    delete engine;
    delete runtime;
  }
};
#endif

namespace
{

#ifdef DETECTION_PKG_USE_TENSORRT
class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT] " << msg << std::endl;
    }
  }
};

TrtLogger g_trt_logger;

bool file_exists(const std::string & path)
{
  return !path.empty() && std::filesystem::exists(path);
}

bool save_engine(const std::string & path, const nvinfer1::IHostMemory & plan)
{
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  out.write(reinterpret_cast<const char *>(plan.data()), static_cast<std::streamsize>(plan.size()));
  return out.good();
}

std::vector<char> load_engine_file(const std::string & path)
{
  std::ifstream in(path, std::ios::binary | std::ios::ate);
  if (!in) {
    return {};
  }
  const auto size = in.tellg();
  in.seekg(0, std::ios::beg);
  std::vector<char> buffer(static_cast<std::size_t>(size));
  in.read(buffer.data(), size);
  return buffer;
}

bool init_trt_state(YoloInference::TrtState & state, const std::string & engine_path)
{
  if (!file_exists(engine_path)) {
    return false;
  }
  const auto bytes = load_engine_file(engine_path);
  if (bytes.empty()) {
    return false;
  }
  state.runtime = nvinfer1::createInferRuntime(g_trt_logger);
  if (!state.runtime) {
    return false;
  }
  state.engine = state.runtime->deserializeCudaEngine(bytes.data(), bytes.size());
  if (!state.engine) {
    return false;
  }
  state.context = state.engine->createExecutionContext();
  if (!state.context) {
    return false;
  }
  cudaStreamCreate(&state.stream);

  const int nb = state.engine->getNbIOTensors();
  for (int i = 0; i < nb; ++i) {
    const char * name = state.engine->getIOTensorName(i);
    const auto mode = state.engine->getTensorIOMode(name);
    const auto dims = state.engine->getTensorShape(name);
    std::size_t vol = 1;
    for (int d = 0; d < dims.nbDims; ++d) {
      vol *= static_cast<std::size_t>(std::max<int64_t>(dims.d[d], 1));
    }
    const auto dtype = state.engine->getTensorDataType(name);
    std::size_t elem = (dtype == nvinfer1::DataType::kFLOAT) ? 4 : 1;
    const std::size_t bytes = vol * elem;
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      state.input_bytes = bytes;
      cudaMalloc(&state.device_input, bytes);
      state.context->setTensorAddress(name, state.device_input);
    } else {
      state.output_bytes = bytes;
      cudaMalloc(&state.device_output, bytes);
      state.context->setTensorAddress(name, state.device_output);
      if (dims.nbDims >= 3) {
        state.num_detections = dims.d[dims.nbDims - 2];
        state.det_stride = dims.d[dims.nbDims - 1];
      }
    }
  }
  return state.input_bytes > 0 && state.output_bytes > 0;
}

bool run_trt(YoloInference::TrtState & state, const std::vector<float> & input, std::vector<float> & output)
{
  cudaMemcpyAsync(
    state.device_input, input.data(), state.input_bytes, cudaMemcpyHostToDevice, state.stream);
  if (!state.context->enqueueV3(state.stream)) {
    return false;
  }
  output.resize(state.output_bytes / sizeof(float));
  cudaMemcpyAsync(
    output.data(), state.device_output, state.output_bytes, cudaMemcpyDeviceToHost, state.stream);
  cudaStreamSynchronize(state.stream);
  return true;
}
#endif

}  // namespace

YoloInference::YoloInference(double conf_threshold, int input_size)
: conf_threshold_(conf_threshold), input_size_(input_size)
{
}

YoloInference::~YoloInference() = default;

YoloBackend YoloInference::backend() const { return backend_; }

std::string YoloInference::resolve_model_paths(std::string & engine_path, std::string & onnx_path)
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("detection_pkg");
    engine_path = share + "/simmodel/best.engine";
    onnx_path = share + "/simmodel/best.onnx";
    if (std::filesystem::exists(engine_path) || std::filesystem::exists(onnx_path)) {
      return share;
    }
    const auto alt_engine = share + "/yolo26n.engine";
    const auto alt_onnx = share + "/yolo26n.onnx";
    if (std::filesystem::exists(alt_engine) || std::filesystem::exists(alt_onnx)) {
      engine_path = alt_engine;
      onnx_path = alt_onnx;
    }
    return share;
  } catch (const std::exception &) {
    return {};
  }
}

bool YoloInference::init(const std::string & weights_param)
{
  resolve_model_paths(engine_path_, onnx_path_);
  if (!weights_param.empty()) {
    const std::filesystem::path p(weights_param);
    if (p.extension() == ".engine") {
      engine_path_ = p.string();
    } else if (p.extension() == ".onnx") {
      onnx_path_ = p.string();
      engine_path_ = (p.parent_path() / (p.stem().string() + ".engine")).string();
    }
  }

#ifdef DETECTION_PKG_USE_TENSORRT
  if (file_exists(engine_path_)) {
    if (!trt_) {
      trt_ = std::make_unique<TrtState>();
    }
    if (init_trt_state(*trt_, engine_path_)) {
      backend_ = YoloBackend::TensorRT;
      return true;
    }
  }
#endif

  if (std::filesystem::exists(onnx_path_)) {
    try {
      dnn_net_ = cv::dnn::readNetFromONNX(onnx_path_);
      dnn_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      dnn_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
      dnn_ready_ = true;
      backend_ = YoloBackend::OpenCVDnn;
      return true;
    } catch (const cv::Exception &) {
      dnn_ready_ = false;
    }
  }

  backend_ = YoloBackend::None;
  return false;
}

cv::Mat YoloInference::preprocess(const cv::Mat & bgr, LetterboxInfo & info) const
{
  info.orig_w = bgr.cols;
  info.orig_h = bgr.rows;
  info.scale = std::min(
    static_cast<float>(input_size_) / static_cast<float>(bgr.cols),
    static_cast<float>(input_size_) / static_cast<float>(bgr.rows));
  const int new_w = static_cast<int>(std::round(bgr.cols * info.scale));
  const int new_h = static_cast<int>(std::round(bgr.rows * info.scale));
  info.pad_x = (input_size_ - new_w) / 2;
  info.pad_y = (input_size_ - new_h) / 2;

  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(new_w, new_h));
  cv::Mat padded(input_size_, input_size_, CV_8UC3, cv::Scalar(114, 114, 114));
  resized.copyTo(padded(cv::Rect(info.pad_x, info.pad_y, new_w, new_h)));

  cv::Mat rgb;
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
  cv::Mat float_img;
  rgb.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
  return float_img;
}

DetectionBox YoloInference::map_box_to_original(
  double x1, double y1, double x2, double y2, double conf, int cls,
  const LetterboxInfo & info) const
{
  const auto unpad = [&](double v, int pad) {
      return (v - static_cast<double>(pad)) / static_cast<double>(info.scale);
    };
  DetectionBox box;
  box.x1 = std::clamp(unpad(x1, info.pad_x), 0.0, static_cast<double>(info.orig_w - 1));
  box.y1 = std::clamp(unpad(y1, info.pad_y), 0.0, static_cast<double>(info.orig_h - 1));
  box.x2 = std::clamp(unpad(x2, info.pad_x), 0.0, static_cast<double>(info.orig_w - 1));
  box.y2 = std::clamp(unpad(y2, info.pad_y), 0.0, static_cast<double>(info.orig_h - 1));
  box.confidence = conf;
  box.class_id = cls;
  return box;
}

std::vector<DetectionBox> YoloInference::postprocess(
  const float * output, int num_detections, int det_stride, const LetterboxInfo & info) const
{
  std::vector<DetectionBox> boxes;
  for (int i = 0; i < num_detections; ++i) {
    const float * row = output + i * det_stride;
    const double conf = row[4];
    if (conf < conf_threshold_) {
      continue;
    }
    const int cls = static_cast<int>(row[5]);
    boxes.push_back(map_box_to_original(row[0], row[1], row[2], row[3], conf, cls, info));
  }
  return boxes;
}

std::vector<DetectionBox> YoloInference::detect(const cv::Mat & bgr)
{
  if (bgr.empty() || backend_ == YoloBackend::None) {
    return {};
  }

  LetterboxInfo info;
  const cv::Mat float_img = preprocess(bgr, info);

  if (backend_ == YoloBackend::OpenCVDnn && dnn_ready_) {
    cv::Mat blob = cv::dnn::blobFromImage(
      float_img, 1.0, cv::Size(), cv::Scalar(), false, false, CV_32F);
    dnn_net_.setInput(blob);
    cv::Mat out = dnn_net_.forward();
    const int num_det = out.size[1];
    const int stride = out.size[2];
    return postprocess(out.ptr<float>(), num_det, stride, info);
  }

#ifdef DETECTION_PKG_USE_TENSORRT
  if (backend_ == YoloBackend::TensorRT && trt_) {
    std::vector<cv::Mat> channels(3);
    cv::split(float_img, channels);
    std::vector<float> input(static_cast<std::size_t>(3 * input_size_ * input_size_));
    const std::size_t plane = static_cast<std::size_t>(input_size_ * input_size_);
    for (int c = 0; c < 3; ++c) {
      std::memcpy(
        input.data() + c * plane, channels[static_cast<std::size_t>(c)].ptr<float>(),
        plane * sizeof(float));
    }
    std::vector<float> output;
    if (!run_trt(*trt_, input, output)) {
      return {};
    }
    return postprocess(output.data(), trt_->num_detections, trt_->det_stride, info);
  }
#endif

  return {};
}

}  // namespace detection_pkg
