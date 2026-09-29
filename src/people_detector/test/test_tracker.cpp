#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "people_detector/tracker.hpp"

using people_detector::Detection;
using people_detector::KalmanTracker;
using people_detector::TrackerParams;

namespace
{
Detection person(double x, double y, double vx = 0.0, double score = 0.9)
{
  Detection detection;
  detection.x = x;
  detection.y = y;
  detection.vx = vx;
  detection.score = score;
  detection.length = 0.8;
  detection.width = 0.6;
  detection.height = 1.7;
  return detection;
}
}  // namespace

TEST(Hungarian, FindsGlobalMinimum)
{
  const auto assignment = people_detector::hungarianAssignment(
    {{4.0, 1.0, 3.0}, {2.0, 0.0, 5.0}, {3.0, 2.0, 2.0}}, 100.0);
  EXPECT_EQ(assignment, (std::vector<int>{1, 0, 2}));
}

TEST(Tracker, ConfirmationCoastingExpirationAndStableId)
{
  TrackerParams params;
  params.confirm_hits = 2;
  params.max_coast_frames = 2;
  params.association_gate_m = 3.0;
  KalmanTracker tracker(params);

  EXPECT_TRUE(tracker.update({person(0.0, 0.0)}, 1.0).empty());
  auto tracks = tracker.update({person(0.1, 0.0)}, 1.1);
  ASSERT_EQ(tracks.size(), 1U);
  const int id = tracks.front().id;
  tracks = tracker.update({}, 1.2);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracks.front().id, id);
  EXPECT_EQ(tracker.update({}, 1.3).size(), 1U);
  EXPECT_TRUE(tracker.update({}, 1.4).empty());
}

TEST(Tracker, ScoreHysteresisMaintainsButDoesNotSpawn)
{
  TrackerParams params;
  params.confirm_hits = 1;
  params.spawn_score_threshold = 0.7;
  params.maintain_score_threshold = 0.3;
  params.association_gate_m = 2.0;
  KalmanTracker tracker(params);
  EXPECT_TRUE(tracker.update({person(0.0, 0.0, 0.0, 0.4)}, 1.0).empty());
  auto tracks = tracker.update({person(0.0, 0.0, 0.0, 0.9)}, 1.1);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_EQ(tracker.update({person(0.1, 0.0, 0.0, 0.4)}, 1.2).size(), 1U);
}

TEST(Tracker, CrossingTracksKeepIds)
{
  TrackerParams params;
  params.confirm_hits = 1;
  params.association_gate_m = 2.5;
  params.position_measurement_noise = 0.05;
  params.velocity_measurement_noise = 0.1;
  KalmanTracker tracker(params);

  auto tracks = tracker.update({person(-1.0, 0.0, 1.0), person(1.0, 0.0, -1.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 2U);
  const int right_mover = tracks[0].id;
  for (int step = 1; step <= 10; ++step) {
    const double t = step * 0.2;
    // Reverse detection ordering every frame to exercise global association.
    tracks = tracker.update(
      {person(1.0 - t, 0.0, -1.0), person(-1.0 + t, 0.0, 1.0)}, t);
  }
  const auto found = std::find_if(
    tracks.begin(), tracks.end(),
    [right_mover](const auto & track) {return track.id == right_mover;});
  ASSERT_NE(found, tracks.end());
  EXPECT_GT(found->x, 0.5);
  EXPECT_GT(found->vx, 0.0);
}

TEST(Tracker, CovarianceGrowsDuringOcclusion)
{
  TrackerParams params;
  params.confirm_hits = 1;
  KalmanTracker tracker(params);
  auto tracks = tracker.update({person(0.0, 0.0)}, 0.0);
  ASSERT_EQ(tracks.size(), 1U);
  const double initial = tracks.front().covariance[0];
  tracks = tracker.update({}, 0.1);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_GT(tracks.front().covariance[0], initial);
}

TEST(Tracker, BackwardAndLargeTimestampGapsReset)
{
  TrackerParams params;
  params.confirm_hits = 1;
  params.max_gap_sec = 0.5;
  KalmanTracker tracker(params);
  auto tracks = tracker.update({person(0.0, 0.0)}, 1.0);
  const auto first_id = tracks.front().id;
  tracks = tracker.update({person(0.0, 0.0)}, 0.9);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_NE(tracks.front().id, first_id);
  const auto second_id = tracks.front().id;
  tracks = tracker.update({person(0.0, 0.0)}, 2.0);
  ASSERT_EQ(tracks.size(), 1U);
  EXPECT_NE(tracks.front().id, second_id);
  EXPECT_EQ(tracker.resetCount(), 2U);
}
