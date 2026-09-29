#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace people_detector
{

struct PipelineParameters
{
  std::string tracking_frame;
  std::string map_frame;
  std::string base_frame;
  int sweep_count{10};
  int max_obstacles{16};
  int confirm_hits{3};
  int max_coast_frames{6};
  double score_threshold{0.5};
  double maintain_score_threshold{0.3};
  double reset_gap_sec{1.0};
  double intensity_scale{1.0};
  double tf_timeout_sec{0.05};
};

inline void validatePipelineParameters(const PipelineParameters & parameters)
{
  if (parameters.tracking_frame.empty() || parameters.map_frame.empty() ||
    parameters.base_frame.empty())
  {
    throw std::invalid_argument("tracking_frame, map_frame, and base_frame must be non-empty");
  }
  if (parameters.sweep_count < 1 || parameters.sweep_count > 10) {
    throw std::invalid_argument("sweep_count must be in [1, 10]");
  }
  if (parameters.score_threshold < 0.0 || parameters.maintain_score_threshold < 0.0 ||
    parameters.maintain_score_threshold > parameters.score_threshold)
  {
    throw std::invalid_argument(
            "thresholds must satisfy 0 <= maintain_score_threshold <= score_threshold");
  }
  if (parameters.max_obstacles < 0 || parameters.confirm_hits < 1 ||
    parameters.max_coast_frames < 0 || parameters.reset_gap_sec <= 0.0 ||
    parameters.intensity_scale < 0.0 || parameters.tf_timeout_sec < 0.0)
  {
    throw std::invalid_argument("invalid negative/zero CenterPoint tracker parameter");
  }
}

inline std::filesystem::path validateModelAssets(
  const std::string & plan_path, const std::string & scn_path, int tensorrt_major,
  bool require_versioned_engine)
{
  namespace fs = std::filesystem;
  std::error_code error;
  if (plan_path.empty() || !fs::exists(plan_path, error)) {
    throw std::runtime_error(
            "CenterPoint TensorRT engine is missing. Run "
            "'./container shell' and then './scripts/generate_centerpoint_engine.sh' "
            "from /nav_ws.");
  }
  const auto resolved_plan = fs::canonical(plan_path, error);
  if (error || !fs::is_regular_file(resolved_plan)) {
    throw std::runtime_error("CenterPoint TensorRT engine path is invalid: " + plan_path);
  }
  error.clear();
  if (scn_path.empty() || !fs::exists(scn_path, error)) {
    throw std::runtime_error(
            "CenterPoint sparse-convolution model is missing: " + scn_path +
            ". Initialize the src/Lidar_AI_Solution submodule.");
  }
  const auto resolved_scn = fs::canonical(scn_path, error);
  if (error || !fs::is_regular_file(resolved_scn)) {
    throw std::runtime_error("CenterPoint sparse-convolution model path is invalid: " + scn_path);
  }
  if (require_versioned_engine) {
    const std::string expected = ".trt" + std::to_string(tensorrt_major) + ".";
    if (resolved_plan.filename().string().find(expected) == std::string::npos) {
      throw std::runtime_error(
              "TensorRT engine '" + resolved_plan.filename().string() +
              "' is not tagged for installed TensorRT " + std::to_string(tensorrt_major) +
              ". Regenerate it with scripts/generate_centerpoint_engine.sh.");
    }
  }
  return resolved_plan;
}

}  // namespace people_detector
