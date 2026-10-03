#include "detection_pkg/trt_engine_builder.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

#ifdef DETECTION_PKG_USE_TENSORRT
#include <NvInfer.h>
#include <NvOnnxParser.h>
#endif

namespace detection_pkg
{

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

TrtLogger g_logger;

bool save_engine(const std::string & path, const nvinfer1::IHostMemory & plan)
{
  std::ofstream out(path, std::ios::binary);
  if (!out) {
    return false;
  }
  out.write(reinterpret_cast<const char *>(plan.data()), static_cast<std::streamsize>(plan.size()));
  return out.good();
}
#endif

}  // namespace

bool build_engine_from_onnx(
  const std::string & onnx_path,
  const std::string & engine_path,
  bool fp16,
  std::size_t workspace_mib)
{
#ifndef DETECTION_PKG_USE_TENSORRT
  (void)onnx_path;
  (void)engine_path;
  (void)fp16;
  (void)workspace_mib;
  std::cerr << "TensorRT support not compiled (USE_TENSORRT=OFF)\n";
  return false;
#else
  if (!std::filesystem::exists(onnx_path)) {
    std::cerr << "ONNX not found: " << onnx_path << '\n';
    return false;
  }

  auto builder = std::unique_ptr<nvinfer1::IBuilder>(
    nvinfer1::createInferBuilder(g_logger));
  if (!builder) {
    return false;
  }

  const auto explicit_batch = 1U << static_cast<uint32_t>(
    nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
    builder->createNetworkV2(explicit_batch));
  auto parser = std::unique_ptr<nvonnxparser::IParser>(
    nvonnxparser::createParser(*network, g_logger));
  if (!parser->parseFromFile(
      onnx_path.c_str(),
      static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
  {
    std::cerr << "ONNX parse failed: " << onnx_path << '\n';
    return false;
  }

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(builder->createBuilderConfig());
  config->setMemoryPoolLimit(
    nvinfer1::MemoryPoolType::kWORKSPACE,
    workspace_mib * (1ULL << 20));
    if (fp16 && builder->platformHasFastFp16()) {
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
  }

  const int nb_inputs = network->getNbInputs();
  bool needs_profile = false;
  for (int i = 0; i < nb_inputs; ++i) {
    const auto dims = network->getInput(i)->getDimensions();
    for (int d = 0; d < dims.nbDims; ++d) {
      if (dims.d[d] < 0) {
        needs_profile = true;
        break;
      }
    }
  }
  if (needs_profile) {
    auto * profile = builder->createOptimizationProfile();
    for (int i = 0; i < nb_inputs; ++i) {
      const char * name = network->getInput(i)->getName();
      const auto dims = network->getInput(i)->getDimensions();
      nvinfer1::Dims fixed = dims;
      for (int d = 0; d < fixed.nbDims; ++d) {
        if (fixed.d[d] < 0) {
          fixed.d[d] = (d == 0) ? 1 : fixed.d[d];  // batch -> 1
        }
      }
      if (fixed.nbDims == 4 && fixed.d[0] < 0) {
        fixed.d[0] = 1;
      }
      profile->setDimensions(name, nvinfer1::OptProfileSelector::kMIN, fixed);
      profile->setDimensions(name, nvinfer1::OptProfileSelector::kOPT, fixed);
      profile->setDimensions(name, nvinfer1::OptProfileSelector::kMAX, fixed);
    }
    config->addOptimizationProfile(profile);
  }

  auto plan = std::unique_ptr<nvinfer1::IHostMemory>(
    builder->buildSerializedNetwork(*network, *config));
  if (!plan) {
    std::cerr << "Engine build failed: " << onnx_path << '\n';
    return false;
  }

  std::filesystem::create_directories(std::filesystem::path(engine_path).parent_path());
  if (!save_engine(engine_path, *plan)) {
    std::cerr << "Failed to write engine: " << engine_path << '\n';
    return false;
  }
  return true;
#endif
}

}  // namespace detection_pkg
