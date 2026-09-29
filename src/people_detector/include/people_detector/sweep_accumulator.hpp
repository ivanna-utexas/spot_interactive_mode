#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace people_detector
{

struct Point4
{
  float x{0.0F};
  float y{0.0F};
  float z{0.0F};
  float intensity{0.0F};
};

struct RigidTransform
{
  std::array<double, 9> rotation{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> translation{0.0, 0.0, 0.0};

  Point4 apply(const Point4 & point) const
  {
    return {
      static_cast<float>(
        rotation[0] * point.x + rotation[1] * point.y + rotation[2] * point.z +
        translation[0]),
      static_cast<float>(
        rotation[3] * point.x + rotation[4] * point.y + rotation[5] * point.z +
        translation[1]),
      static_cast<float>(
        rotation[6] * point.x + rotation[7] * point.y + rotation[8] * point.z +
        translation[2]),
      point.intensity};
  }

  RigidTransform inverse() const
  {
    RigidTransform result;
    result.rotation = {
      rotation[0], rotation[3], rotation[6],
      rotation[1], rotation[4], rotation[7],
      rotation[2], rotation[5], rotation[8]};
    for (int row = 0; row < 3; ++row) {
      result.translation[row] = -(
        result.rotation[row * 3] * translation[0] +
        result.rotation[row * 3 + 1] * translation[1] +
        result.rotation[row * 3 + 2] * translation[2]);
    }
    return result;
  }

  RigidTransform operator*(const RigidTransform & rhs) const
  {
    RigidTransform result;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        result.rotation[row * 3 + col] = 0.0;
        for (int k = 0; k < 3; ++k) {
          result.rotation[row * 3 + col] +=
            rotation[row * 3 + k] * rhs.rotation[k * 3 + col];
        }
      }
    }
    for (int row = 0; row < 3; ++row) {
      result.translation[row] = translation[row];
      for (int k = 0; k < 3; ++k) {
        result.translation[row] += rotation[row * 3 + k] * rhs.translation[k];
      }
    }
    return result;
  }
};

class SweepAccumulator
{
public:
  SweepAccumulator(std::size_t sweep_count, std::size_t point_limit, double max_gap_sec)
  : sweep_count_(sweep_count), point_limit_(point_limit), max_gap_sec_(max_gap_sec) {}

  bool add(
    std::vector<Point4> points, const RigidTransform & tracking_from_sensor,
    double stamp_sec)
  {
    bool reset = false;
    if (!sweeps_.empty()) {
      const double gap = stamp_sec - sweeps_.back().stamp_sec;
      if (!std::isfinite(gap) || gap <= 0.0 || gap > max_gap_sec_) {
        sweeps_.clear();
        reset = true;
      }
    }
    sweeps_.push_back({std::move(points), tracking_from_sensor, stamp_sec});
    while (sweeps_.size() > sweep_count_) {
      sweeps_.pop_front();
    }
    return reset;
  }

  std::size_t size() const {return sweeps_.size();}
  void clear() {sweeps_.clear();}

  // Returns [x,y,z,intensity,time_lag], newest sweep first. This ordering means
  // the point cap always discards the oldest observations first.
  std::vector<float> packNewestFrame(
    const RigidTransform & tracking_from_newest_sensor, double newest_stamp_sec,
    float z_offset, float intensity_scale) const
  {
    std::vector<float> packed;
    packed.reserve(std::min(point_limit_, totalPoints()) * 5);
    const auto newest_sensor_from_tracking = tracking_from_newest_sensor.inverse();
    for (auto sweep = sweeps_.rbegin(); sweep != sweeps_.rend(); ++sweep) {
      const auto newest_sensor_from_old_sensor =
        newest_sensor_from_tracking * sweep->tracking_from_sensor;
      const float lag = static_cast<float>(newest_stamp_sec - sweep->stamp_sec);
      for (const auto & raw_point : sweep->points) {
        if (packed.size() / 5 >= point_limit_) {
          return packed;
        }
        auto point = newest_sensor_from_old_sensor.apply(raw_point);
        point.z += z_offset;
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z) || !std::isfinite(point.intensity))
        {
          continue;
        }
        packed.push_back(point.x);
        packed.push_back(point.y);
        packed.push_back(point.z);
        packed.push_back(point.intensity * intensity_scale);
        packed.push_back(lag);
      }
    }
    return packed;
  }

private:
  struct Sweep
  {
    std::vector<Point4> points;
    RigidTransform tracking_from_sensor;
    double stamp_sec{0.0};
  };

  std::size_t totalPoints() const
  {
    std::size_t count = 0;
    for (const auto & sweep : sweeps_) {
      count += sweep.points.size();
    }
    return count;
  }

  std::size_t sweep_count_;
  std::size_t point_limit_;
  double max_gap_sec_;
  std::deque<Sweep> sweeps_;
};

}  // namespace people_detector
