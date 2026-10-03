#include "detection_pkg/trt_osnet.hpp"

#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <opencv2/imgproc.hpp>

#ifdef DETECTION_PKG_USE_TENSORRT
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#endif

namespace detection_pkg
{

#ifdef DETECTION_PKG_USE_TENSORRT
namespace
{

class TrtLogger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char * msg) noexcept override
  {
    if (severity <= Severity::kWARNING) {
      std::cerr << "[TensorRT-OSNet] " << msg << std::endl;
    }
  }
};

TrtLogger g_osnet_logger;

struct OsnetTrtStateImpl
{
  nvinfer1::IRuntime * runtime{nullptr};
  nvinfer1::ICudaEngine * engine{nullptr};
  nvinfer1::IExecutionContext * context{nullptr};
  cudaStream_t stream{};
  void * device_input{nullptr};
  void * device_output{nullptr};
  std::size_t input_bytes{0};
  std::size_t output_bytes{0};
  int embed_dim{256};

  ~OsnetTrtStateImpl()
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

bool init_osnet_trt(OsnetTrtStateImpl & state, const std::string & engine_path)
{
  if (!std::filesystem::exists(engine_path)) {
    return false;
  }
  std::ifstream in(engine_path, std::ios::binary | std::ios::ate);
  if (!in) {
    return false;
  }
  const auto size = in.tellg();
  in.seekg(0);
  std::vector<char> bytes(static_cast<std::size_t>(size));
  in.read(bytes.data(), size);
  state.runtime = nvinfer1::createInferRuntime(g_osnet_logger);
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
    const std::size_t bytes_count = vol * sizeof(float);
    if (mode == nvinfer1::TensorIOMode::kINPUT) {
      state.input_bytes = bytes_count;
      cudaMalloc(&state.device_input, bytes_count);
      state.context->setTensorAddress(name, state.device_input);
    } else {
      state.output_bytes = bytes_count;
      state.embed_dim = static_cast<int>(vol);
      cudaMalloc(&state.device_output, bytes_count);
      state.context->setTensorAddress(name, state.device_output);
    }
  }
  return state.input_bytes > 0 && state.output_bytes > 0;
}

}  // namespace

struct OsnetTrtEmbedding::TrtState : OsnetTrtStateImpl
{
};
#endif

std::string OsnetTrtEmbedding::resolve_model_path()
{
  try {
    const auto share = ament_index_cpp::get_package_share_directory("detection_pkg");
    const auto engine = share + "/simmodel/osnet_ain_x0_25.engine";
    if (std::filesystem::exists(engine)) {
      return engine;
    }
    const auto onnx = share + "/simmodel/osnet_ain_x0_25.onnx";
    if (std::filesystem::exists(onnx)) {
      return onnx;
    }
  } catch (const std::exception &) {
  }
  return {};
}

OsnetTrtEmbedding::OsnetTrtEmbedding(double cosine_min, double ema_alpha)
: cosine_min_(cosine_min), ema_alpha_(ema_alpha)
{
#ifdef DETECTION_PKG_USE_TENSORRT
  try {
    const auto share = ament_index_cpp::get_package_share_directory("detection_pkg");
    const std::string engine = share + "/simmodel/osnet_ain_x0_25.engine";
    auto state = std::make_unique<TrtState>();
    ready_ = init_osnet_trt(*state, engine);
    if (ready_) {
      trt_ = std::move(state);
    }
  } catch (const std::exception &) {
    ready_ = false;
  }
#else
  (void)cosine_min_;
  (void)ema_alpha_;
#endif
}

OsnetTrtEmbedding::~OsnetTrtEmbedding() = default;

bool OsnetTrtEmbedding::available() const { return ready_; }
std::size_t OsnetTrtEmbedding::size() const { return entries_.size(); }

std::vector<float> OsnetTrtEmbedding::normalize(const std::vector<float> & vec)
{
  double norm = 0.0;
  for (float v : vec) {
    norm += static_cast<double>(v) * static_cast<double>(v);
  }
  norm = std::sqrt(norm);
  if (norm < 1e-8) {
    return vec;
  }
  std::vector<float> out(vec.size());
  for (std::size_t i = 0; i < vec.size(); ++i) {
    out[i] = static_cast<float>(vec[i] / norm);
  }
  return out;
}

double OsnetTrtEmbedding::cosine(const std::vector<float> & a, const std::vector<float> & b)
{
  const std::size_t n = std::min(a.size(), b.size());
  double sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    sum += static_cast<double>(a[i]) * static_cast<double>(b[i]);
  }
  return sum;
}

std::optional<std::vector<float>> OsnetTrtEmbedding::embed(const cv::Mat & bgr)
{
#ifndef DETECTION_PKG_USE_TENSORRT
  (void)bgr;
  return std::nullopt;
#else
  if (!ready_ || !trt_) {
    return std::nullopt;
  }
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(128, 256));
  cv::Mat rgb;
  cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);
  cv::Mat float_img;
  rgb.convertTo(float_img, CV_32FC3, 1.0 / 255.0);
  std::vector<cv::Mat> channels(3);
  cv::split(float_img, channels);
  const int h = 256;
  const int w = 128;
  std::vector<float> input(static_cast<std::size_t>(3 * h * w));
  const std::size_t plane = static_cast<std::size_t>(h * w);
  for (int c = 0; c < 3; ++c) {
    std::memcpy(
      input.data() + c * plane, channels[static_cast<std::size_t>(c)].ptr<float>(),
      plane * sizeof(float));
  }
  auto * state = static_cast<OsnetTrtStateImpl *>(trt_.get());
  cudaMemcpyAsync(
    state->device_input, input.data(), state->input_bytes, cudaMemcpyHostToDevice, state->stream);
  if (!state->context->enqueueV3(state->stream)) {
    return std::nullopt;
  }
  std::vector<float> output(static_cast<std::size_t>(state->embed_dim));
  cudaMemcpyAsync(
    output.data(), state->device_output, state->output_bytes, cudaMemcpyDeviceToHost, state->stream);
  cudaStreamSynchronize(state->stream);
  return normalize(output);
#endif
}

std::pair<std::optional<int>, double> OsnetTrtEmbedding::query(const cv::Mat & bgr)
{
  if (entries_.empty()) {
    return {std::nullopt, 0.0};
  }
  const auto vec = embed(bgr);
  if (!vec.has_value()) {
    return {std::nullopt, 0.0};
  }
  std::optional<int> best_idx;
  double best_score = -1.0;
  for (const auto & [idx, entry] : entries_) {
    const double score = cosine(vec.value(), entry);
    if (score > best_score) {
      best_score = score;
      best_idx = idx;
    }
  }
  if (best_idx.has_value() && best_score >= cosine_min_) {
    return {best_idx, best_score};
  }
  return {std::nullopt, std::max(best_score, 0.0)};
}

void OsnetTrtEmbedding::register_waypoint(int waypoint_index, const cv::Mat & bgr)
{
  const auto vec = embed(bgr);
  if (!vec.has_value()) {
    return;
  }
  if (entries_.count(waypoint_index) > 0) {
    const auto & old = entries_.at(waypoint_index);
    std::vector<float> blended(old.size());
    for (std::size_t i = 0; i < blended.size(); ++i) {
      blended[i] = static_cast<float>((1.0 - ema_alpha_) * old[i] + ema_alpha_ * vec->at(i));
    }
    entries_[waypoint_index] = normalize(blended);
  } else {
    entries_[waypoint_index] = vec.value();
  }
}

}  // namespace detection_pkg
