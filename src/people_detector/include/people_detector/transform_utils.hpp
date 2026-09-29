#pragma once

#include <array>

#include "people_detector/sweep_accumulator.hpp"

namespace people_detector
{

inline std::array<double, 3> rotateVector(
  const RigidTransform & transform, double x, double y, double z = 0.0)
{
  return {
    transform.rotation[0] * x + transform.rotation[1] * y + transform.rotation[2] * z,
    transform.rotation[3] * x + transform.rotation[4] * y + transform.rotation[5] * z,
    transform.rotation[6] * x + transform.rotation[7] * y + transform.rotation[8] * z};
}

inline std::array<double, 9> rotateCovariance(
  const RigidTransform & transform, const std::array<double, 9> & covariance)
{
  std::array<double, 9> intermediate{};
  std::array<double, 9> result{};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      for (int k = 0; k < 3; ++k) {
        intermediate[row * 3 + col] +=
          transform.rotation[row * 3 + k] * covariance[k * 3 + col];
      }
    }
  }
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) {
      for (int k = 0; k < 3; ++k) {
        result[row * 3 + col] +=
          intermediate[row * 3 + k] * transform.rotation[col * 3 + k];
      }
    }
  }
  return result;
}

}  // namespace people_detector
