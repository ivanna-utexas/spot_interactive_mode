#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "people_detector/sweep_accumulator.hpp"

using people_detector::Point4;
using people_detector::RigidTransform;
using people_detector::SweepAccumulator;

TEST(SweepAccumulator, EncodesTenMotionCompensatedSweeps)
{
  SweepAccumulator accumulator(10, 300000, 1.0);
  for (int index = 0; index < 10; ++index) {
    RigidTransform tracking_from_sensor;
    tracking_from_sensor.translation[0] = index * 0.1;
    EXPECT_FALSE(accumulator.add({Point4{1.0F, 0.0F, 0.0F, 1.0F}},
      tracking_from_sensor, index * 0.1));
  }
  RigidTransform newest;
  newest.translation[0] = 0.9;
  const auto packed = accumulator.packNewestFrame(newest, 0.9, 0.0F, 1.0F);
  ASSERT_EQ(packed.size(), 50U);
  EXPECT_NEAR(packed[0], 1.0, 1.0e-6);   // newest remains unchanged
  EXPECT_NEAR(packed[4], 0.0, 1.0e-6);
  EXPECT_NEAR(packed[45], 0.1, 1.0e-6);  // oldest compensates robot translation
  EXPECT_NEAR(packed[49], 0.9, 1.0e-6);
}

TEST(SweepAccumulator, KeepsNewestPointsAtLimitAndHandlesMissingIntensity)
{
  SweepAccumulator accumulator(2, 2, 1.0);
  RigidTransform identity;
  accumulator.add({Point4{1, 0, 0, 0}, Point4{2, 0, 0, 0}}, identity, 0.0);
  accumulator.add({Point4{3, 0, 0, 0}, Point4{4, 0, 0, 0}}, identity, 0.1);
  const auto packed = accumulator.packNewestFrame(identity, 0.1, 0.0F, 1.0F);
  ASSERT_EQ(packed.size(), 10U);
  EXPECT_FLOAT_EQ(packed[0], 3.0F);
  EXPECT_FLOAT_EQ(packed[5], 4.0F);
  EXPECT_FLOAT_EQ(packed[3], 0.0F);
}

TEST(SweepAccumulator, DropsInvalidPoints)
{
  SweepAccumulator accumulator(1, 10, 1.0);
  RigidTransform identity;
  accumulator.add(
    {Point4{1, 2, 3, 4},
      Point4{std::numeric_limits<float>::quiet_NaN(), 0, 0, 0}},
    identity, 0.0);
  const auto packed = accumulator.packNewestFrame(identity, 0.0, 1.0F, 2.0F);
  ASSERT_EQ(packed.size(), 5U);
  EXPECT_FLOAT_EQ(packed[2], 4.0F);
  EXPECT_FLOAT_EQ(packed[3], 8.0F);
}

TEST(SweepAccumulator, ResetsOnBackwardTimeAndLargeGap)
{
  SweepAccumulator accumulator(10, 100, 0.5);
  RigidTransform identity;
  EXPECT_FALSE(accumulator.add({Point4{}}, identity, 1.0));
  EXPECT_TRUE(accumulator.add({Point4{}}, identity, 0.9));
  EXPECT_EQ(accumulator.size(), 1U);
  EXPECT_TRUE(accumulator.add({Point4{}}, identity, 2.0));
  EXPECT_EQ(accumulator.size(), 1U);
}
