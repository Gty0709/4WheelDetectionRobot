#include "detection_pkg/trt_engine_builder.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

namespace
{

struct Job
{
  std::string onnx;
  std::string engine;
  std::size_t workspace_mib;
};

void print_usage(const char * argv0)
{
  std::cerr
    << "Usage: " << argv0 << " [--fp32] [--workspace-mib N] <onnx> [engine]\n"
    << "       " << argv0 << " --all   # build simmodel + yolo26n engines\n";
}

std::filesystem::path ws_root()
{
  const char * env = std::getenv("ROBOTHOMEWORK_WS");
  if (env && env[0]) {
    return std::filesystem::path(env);
  }
  std::filesystem::path p = std::filesystem::current_path();
  for (int i = 0; i < 8; ++i) {
    if (std::filesystem::exists(p / "src" / "detection_pkg" / "simmodel")) {
      return p;
    }
    if (!p.has_parent_path()) {
      break;
    }
    p = p.parent_path();
  }
  return std::filesystem::current_path();
}

int run_all(bool fp16)
{
  const auto root = ws_root();
  const auto simmodel = root / "src" / "detection_pkg" / "simmodel";
  const std::vector<Job> jobs = {
    {simmodel.string() + "/best.onnx", simmodel.string() + "/best.engine", 2048},
    {
      (root / "src" / "detection_pkg" / "yolo26n.onnx").string(),
      (root / "src" / "detection_pkg" / "yolo26n.engine").string(),
      1024,
    },
    {
      simmodel.string() + "/osnet_ain_x0_25.onnx",
      simmodel.string() + "/osnet_ain_x0_25.engine",
      512,
    },
  };

  int failures = 0;
  for (const auto & job : jobs) {
    if (!std::filesystem::exists(job.onnx)) {
      std::cout << "SKIP (no onnx): " << job.onnx << '\n';
      continue;
    }
    if (std::filesystem::exists(job.engine)) {
      std::cout << "Already exists: " << job.engine << '\n';
      continue;
    }
    std::cout << "Building " << job.onnx << " -> " << job.engine << " ...\n";
    if (!detection_pkg::build_engine_from_onnx(job.onnx, job.engine, fp16, job.workspace_mib)) {
      ++failures;
    } else {
      std::cout << "Done: " << job.engine << '\n';
    }
  }
  return failures == 0 ? 0 : 1;
}

}  // namespace

int main(int argc, char ** argv)
{
  bool fp16 = true;
  std::size_t workspace_mib = 2048;
  std::vector<std::string> args;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--fp32") {
      fp16 = false;
    } else if (a == "--fp16") {
      fp16 = true;
    } else if (a == "--all") {
      return run_all(fp16);
    } else if (a == "--workspace-mib" && i + 1 < argc) {
      workspace_mib = static_cast<std::size_t>(std::stoul(argv[++i]));
    } else if (a == "-h" || a == "--help") {
      print_usage(argv[0]);
      return 0;
    } else {
      args.push_back(a);
    }
  }

  if (args.empty()) {
    return run_all(fp16);
  }

  const std::string onnx = args[0];
  const std::string engine = args.size() > 1 ? args[1] : (onnx.substr(0, onnx.size() - 5) + ".engine");
  if (!detection_pkg::build_engine_from_onnx(onnx, engine, fp16, workspace_mib)) {
    return 1;
  }
  std::cout << "Wrote " << engine << '\n';
  return 0;
}
