#pragma once

#include <string>

namespace detection_pkg
{

/** Build a TensorRT serialized engine from ONNX. Returns true on success. */
bool build_engine_from_onnx(
  const std::string & onnx_path,
  const std::string & engine_path,
  bool fp16 = true,
  std::size_t workspace_mib = 2048);

}  // namespace detection_pkg
