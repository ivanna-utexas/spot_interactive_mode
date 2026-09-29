#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "people_detector/transform_utils.hpp"
#include "people_detector/validation.hpp"

TEST(Validation, RejectsInvalidParameters)
{
  people_detector::PipelineParameters parameters;
  parameters.tracking_frame = "odom";
  parameters.map_frame = "map";
  parameters.base_frame = "base_link";
  EXPECT_NO_THROW(people_detector::validatePipelineParameters(parameters));
  parameters.sweep_count = 0;
  EXPECT_THROW(
    people_detector::validatePipelineParameters(parameters), std::invalid_argument);
  parameters.sweep_count = 10;
  parameters.maintain_score_threshold = 0.8;
  EXPECT_THROW(
    people_detector::validatePipelineParameters(parameters), std::invalid_argument);
}

TEST(Validation, RejectsMissingAndIncompatibleAssets)
{
  const auto root = std::filesystem::temp_directory_path() /
    "people_detector_asset_validation";
  std::filesystem::create_directories(root);
  const auto scn = root / "centerpoint.scn.onnx";
  const auto wrong_plan = root / "rpn_centerhead_sim.trt9.0.0.plan";
  std::ofstream(scn.string()) << "scn";
  std::ofstream(wrong_plan.string()) << "plan";
  EXPECT_THROW(
    people_detector::validateModelAssets(
      (root / "missing.plan").string(), scn.string(), 10, true),
    std::runtime_error);
  EXPECT_THROW(
    people_detector::validateModelAssets(
      wrong_plan.string(), scn.string(), 10, true),
    std::runtime_error);
  std::filesystem::remove_all(root);
}

TEST(Transforms, RotatesPositionsVelocitiesAndCovariance)
{
  people_detector::RigidTransform transform;
  transform.rotation = {0.0, -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  transform.translation = {10.0, 20.0, 0.0};
  const auto point = transform.apply({2.0F, 3.0F, 4.0F, 0.0F});
  EXPECT_FLOAT_EQ(point.x, 7.0F);
  EXPECT_FLOAT_EQ(point.y, 22.0F);
  EXPECT_FLOAT_EQ(point.z, 4.0F);
  const auto velocity = people_detector::rotateVector(transform, 2.0, 3.0);
  EXPECT_DOUBLE_EQ(velocity[0], -3.0);
  EXPECT_DOUBLE_EQ(velocity[1], 2.0);
  const std::array<double, 9> covariance{
    4.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 9.0};
  const auto rotated = people_detector::rotateCovariance(transform, covariance);
  EXPECT_NEAR(rotated[0], 1.0, 1.0e-12);
  EXPECT_NEAR(rotated[4], 4.0, 1.0e-12);
  EXPECT_NEAR(rotated[8], 9.0, 1.0e-12);
}
