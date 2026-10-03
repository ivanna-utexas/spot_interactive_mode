#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace people_detector
{

// Rejects CenterPoint pedestrian boxes whose geometry cannot be a person. The
// nuScenes model is out of domain on a low-mounted VLP-16 indoors, so furniture
// is often labelled "pedestrian"; most of those boxes are too short, too wide,
// or too far away for a 16-beam sensor to have resolved a body.
struct DetectionFilterParams
{
  double min_height_m{1.0};
  double max_height_m{2.2};
  // Larger of the box length and width.
  double max_footprint_m{1.2};
  // Horizontal range from the LiDAR. Zero disables the range limit.
  double max_range_m{10.0};
};

inline void validateDetectionFilterParams(const DetectionFilterParams & params)
{
  if (!(params.min_height_m >= 0.0) || !(params.max_height_m > params.min_height_m) ||
    !(params.max_footprint_m > 0.0) || !(params.max_range_m >= 0.0))
  {
    throw std::invalid_argument(
            "detection filter requires 0 <= min_height_m < max_height_m, "
            "max_footprint_m > 0, and max_range_m >= 0");
  }
}

// Box center (x, y) is in the sensor frame; dimensions are in meters.
inline bool isPlausiblePerson(
  const DetectionFilterParams & params, double x, double y, double length,
  double width, double height)
{
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(length) ||
    !std::isfinite(width) || !std::isfinite(height))
  {
    return false;
  }
  if (height < params.min_height_m || height > params.max_height_m) {
    return false;
  }
  if (std::max(length, width) > params.max_footprint_m) {
    return false;
  }
  return params.max_range_m <= 0.0 || std::hypot(x, y) <= params.max_range_m;
}

}  // namespace people_detector
