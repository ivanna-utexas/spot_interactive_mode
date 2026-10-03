#include <gtest/gtest.h>

#include <limits>
#include <stdexcept>

#include "people_detector/detection_filter.hpp"

using people_detector::DetectionFilterParams;
using people_detector::isPlausiblePerson;

TEST(DetectionFilter, AcceptsPersonShapedBox)
{
  const DetectionFilterParams params;
  EXPECT_TRUE(isPlausiblePerson(params, 3.0, 1.0, 0.7, 0.6, 1.75));
}

TEST(DetectionFilter, RejectsImplausibleHeightFootprintAndRange)
{
  const DetectionFilterParams params;
  // Chair/table height.
  EXPECT_FALSE(isPlausiblePerson(params, 3.0, 0.0, 0.6, 0.6, 0.8));
  // Taller than any person.
  EXPECT_FALSE(isPlausiblePerson(params, 3.0, 0.0, 0.6, 0.6, 2.5));
  // Couch-sized footprint, in either dimension.
  EXPECT_FALSE(isPlausiblePerson(params, 3.0, 0.0, 1.8, 0.6, 1.7));
  EXPECT_FALSE(isPlausiblePerson(params, 3.0, 0.0, 0.6, 1.8, 1.7));
  // Beyond max_range_m horizontally.
  EXPECT_FALSE(isPlausiblePerson(params, 8.0, 8.0, 0.7, 0.6, 1.7));
  EXPECT_FALSE(isPlausiblePerson(
      params, std::numeric_limits<double>::quiet_NaN(), 0.0, 0.7, 0.6, 1.7));
}

TEST(DetectionFilter, ZeroRangeDisablesRangeLimit)
{
  DetectionFilterParams params;
  params.max_range_m = 0.0;
  EXPECT_TRUE(isPlausiblePerson(params, 40.0, 0.0, 0.7, 0.6, 1.7));
}

TEST(DetectionFilter, ValidatesParameters)
{
  DetectionFilterParams params;
  EXPECT_NO_THROW(people_detector::validateDetectionFilterParams(params));
  params.min_height_m = 2.5;
  EXPECT_THROW(
    people_detector::validateDetectionFilterParams(params), std::invalid_argument);
  params = DetectionFilterParams();
  params.max_range_m = -1.0;
  EXPECT_THROW(
    people_detector::validateDetectionFilterParams(params), std::invalid_argument);
}
