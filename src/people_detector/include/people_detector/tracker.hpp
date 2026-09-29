#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace people_detector
{

struct Detection
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double length{0.0};
  double width{0.0};
  double height{0.0};
  double yaw{0.0};
  double score{0.0};
};

struct TrackState
{
  int32_t id{0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double vx{0.0};
  double vy{0.0};
  double length{0.0};
  double width{0.0};
  double height{0.0};
  double yaw{0.0};
  double score{0.0};
  std::array<double, 16> covariance{};
  int hits{0};
  int misses{0};
  double age_sec{0.0};
  bool confirmed{false};
};

struct TrackerParams
{
  double association_gate_m{1.5};
  double mahalanobis_gate{9.21};
  double association_velocity_weight{0.5};
  double position_process_noise{1.0};
  double velocity_process_noise{2.0};
  double position_measurement_noise{0.25};
  double velocity_measurement_noise{2.0};
  double spawn_score_threshold{0.5};
  double maintain_score_threshold{0.3};
  double max_gap_sec{1.0};
  int confirm_hits{3};
  int max_coast_frames{6};
};

// Minimum-cost one-to-one assignment. Rows assigned to dummy columns are -1.
inline std::vector<int> hungarianAssignment(
  const std::vector<std::vector<double>> & rectangular_cost,
  double unassigned_cost)
{
  const std::size_t rows = rectangular_cost.size();
  const std::size_t cols = rows == 0 ? 0 : rectangular_cost.front().size();
  // Include explicit dummy columns even for a square real cost matrix, so a
  // forbidden/bad pair never has to displace a valid association.
  const std::size_t n = rows + cols;
  if (n == 0) {
    return {};
  }

  // Classic O(n^3) shortest augmenting-path implementation, 1-indexed.
  std::vector<std::vector<double>> cost(n + 1, std::vector<double>(n + 1, 0.0));
  for (std::size_t i = 0; i < rows; ++i) {
    for (std::size_t j = 0; j < cols; ++j) {
      cost[i + 1][j + 1] = rectangular_cost[i][j];
    }
    for (std::size_t j = cols; j < n; ++j) {
      cost[i + 1][j + 1] = unassigned_cost;
    }
  }

  std::vector<double> u(n + 1), v(n + 1);
  std::vector<std::size_t> p(n + 1), way(n + 1);
  for (std::size_t i = 1; i <= n; ++i) {
    p[0] = i;
    std::size_t j0 = 0;
    std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
    std::vector<bool> used(n + 1, false);
    do {
      used[j0] = true;
      const std::size_t i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      std::size_t j1 = 0;
      for (std::size_t j = 1; j <= n; ++j) {
        if (used[j]) {
          continue;
        }
        const double cur = cost[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (std::size_t j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      const std::size_t j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  std::vector<int> assignment(rows, -1);
  for (std::size_t j = 1; j <= n; ++j) {
    if (p[j] >= 1 && p[j] <= rows && j <= cols &&
      cost[p[j]][j] < unassigned_cost)
    {
      assignment[p[j] - 1] = static_cast<int>(j - 1);
    }
  }
  return assignment;
}

class KalmanTracker
{
public:
  explicit KalmanTracker(const TrackerParams & params = TrackerParams())
  : params_(params) {}

  void configure(const TrackerParams & params) {params_ = params;}
  void setSpawnThreshold(double score) {params_.spawn_score_threshold = score;}
  void reset()
  {
    tracks_.clear();
    have_stamp_ = false;
    ++reset_count_;
  }

  std::size_t resetCount() const {return reset_count_;}
  const std::vector<TrackState> & allTracks() const {return tracks_;}

  std::vector<TrackState> update(const std::vector<Detection> & detections, double stamp_sec)
  {
    double dt = 0.0;
    if (have_stamp_) {
      dt = stamp_sec - last_stamp_sec_;
      if (!std::isfinite(dt) || dt <= 0.0 || dt > params_.max_gap_sec) {
        tracks_.clear();
        ++reset_count_;
        dt = 0.0;
      }
    }
    last_stamp_sec_ = stamp_sec;
    have_stamp_ = true;

    for (auto & track : tracks_) {
      predict(track, dt);
    }

    constexpr double kForbidden = 1.0e9;
    const double unassigned = std::max(params_.mahalanobis_gate, 1.0) + 1.0;
    std::vector<std::vector<double>> costs(
      tracks_.size(), std::vector<double>(detections.size(), kForbidden));
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      for (std::size_t d = 0; d < detections.size(); ++d) {
        if (detections[d].score < params_.maintain_score_threshold) {
          continue;
        }
        const double dx = detections[d].x - tracks_[t].x;
        const double dy = detections[d].y - tracks_[t].y;
        if (std::hypot(dx, dy) > params_.association_gate_m) {
          continue;
        }
        const double sx = tracks_[t].covariance[0] +
          params_.position_measurement_noise;
        const double sy = tracks_[t].covariance[5] +
          params_.position_measurement_noise;
        const double md2 = dx * dx / std::max(sx, 1.0e-9) +
          dy * dy / std::max(sy, 1.0e-9);
        if (md2 <= params_.mahalanobis_gate) {
          const double dvx = detections[d].vx - tracks_[t].vx;
          const double dvy = detections[d].vy - tracks_[t].vy;
          const double velocity_cost = params_.association_velocity_weight *
            (dvx * dvx + dvy * dvy) /
            std::max(params_.velocity_measurement_noise, 1.0e-9);
          costs[t][d] = md2 + velocity_cost;
        }
      }
    }

    const auto assignment = hungarianAssignment(costs, unassigned);
    std::vector<bool> detection_used(detections.size(), false);
    std::vector<TrackState> survivors;
    survivors.reserve(tracks_.size() + detections.size());
    for (std::size_t t = 0; t < tracks_.size(); ++t) {
      auto track = tracks_[t];
      const int d = assignment.empty() ? -1 : assignment[t];
      if (d >= 0 && costs[t][static_cast<std::size_t>(d)] < unassigned) {
        correct(track, detections[static_cast<std::size_t>(d)]);
        detection_used[static_cast<std::size_t>(d)] = true;
        track.hits += 1;
        track.misses = 0;
        track.confirmed = track.confirmed || track.hits >= params_.confirm_hits;
      } else {
        track.misses += 1;
      }
      if (track.misses <= params_.max_coast_frames) {
        survivors.push_back(track);
      }
    }

    for (std::size_t d = 0; d < detections.size(); ++d) {
      if (detection_used[d] || detections[d].score < params_.spawn_score_threshold) {
        continue;
      }
      TrackState track;
      track.id = next_id_++;
      track.x = detections[d].x;
      track.y = detections[d].y;
      track.z = detections[d].z;
      track.vx = detections[d].vx;
      track.vy = detections[d].vy;
      track.length = detections[d].length;
      track.width = detections[d].width;
      track.height = detections[d].height;
      track.yaw = detections[d].yaw;
      track.score = detections[d].score;
      track.hits = 1;
      track.confirmed = params_.confirm_hits <= 1;
      track.covariance[0] = params_.position_measurement_noise;
      track.covariance[5] = params_.position_measurement_noise;
      track.covariance[10] = params_.velocity_measurement_noise;
      track.covariance[15] = params_.velocity_measurement_noise;
      survivors.push_back(track);
    }
    tracks_.swap(survivors);

    std::vector<TrackState> confirmed;
    for (const auto & track : tracks_) {
      if (track.confirmed) {
        confirmed.push_back(track);
      }
    }
    return confirmed;
  }

private:
  static void predictAxis(
    double & position, double & velocity, double & pp, double & pv, double & vp,
    double & vv, double dt, double q_position, double q_velocity)
  {
    position += velocity * dt;
    const double old_pp = pp;
    const double old_pv = pv;
    const double old_vp = vp;
    const double old_vv = vv;
    pp = old_pp + dt * (old_pv + old_vp) + dt * dt * old_vv +
      q_position * std::max(dt, 1.0e-3);
    pv = old_pv + dt * old_vv;
    vp = old_vp + dt * old_vv;
    vv = old_vv + q_velocity * std::max(dt, 1.0e-3);
  }

  void predict(TrackState & track, double dt) const
  {
    predictAxis(
      track.x, track.vx, track.covariance[0], track.covariance[2],
      track.covariance[8], track.covariance[10], dt,
      params_.position_process_noise, params_.velocity_process_noise);
    predictAxis(
      track.y, track.vy, track.covariance[5], track.covariance[7],
      track.covariance[13], track.covariance[15], dt,
      params_.position_process_noise, params_.velocity_process_noise);
    track.age_sec += std::max(0.0, dt);
  }

  static void correctAxis(
    double measurement, double measurement_noise, double & position, double & velocity,
    double & pp, double & pv, double & vp, double & vv)
  {
    const double innovation_variance = pp + measurement_noise;
    const double kp = pp / innovation_variance;
    const double kv = vp / innovation_variance;
    const double residual = measurement - position;
    position += kp * residual;
    velocity += kv * residual;
    const double old_pp = pp;
    const double old_pv = pv;
    pp = (1.0 - kp) * old_pp;
    pv = (1.0 - kp) * old_pv;
    vp -= kv * old_pp;
    vv -= kv * old_pv;
  }

  void correct(TrackState & track, const Detection & detection) const
  {
    correctAxis(
      detection.x, params_.position_measurement_noise, track.x, track.vx,
      track.covariance[0], track.covariance[2], track.covariance[8],
      track.covariance[10]);
    correctAxis(
      detection.y, params_.position_measurement_noise, track.y, track.vy,
      track.covariance[5], track.covariance[7], track.covariance[13],
      track.covariance[15]);
    // The network velocity is noisy, but blending it helps establish direction
    // before position-only CV updates converge.
    const double velocity_weight =
      1.0 / (1.0 + std::max(params_.velocity_measurement_noise, 0.0));
    track.vx = (1.0 - velocity_weight) * track.vx + velocity_weight * detection.vx;
    track.vy = (1.0 - velocity_weight) * track.vy + velocity_weight * detection.vy;
    track.z = detection.z;
    track.length = detection.length;
    track.width = detection.width;
    track.height = detection.height;
    track.yaw = detection.yaw;
    track.score = detection.score;
  }

  TrackerParams params_;
  std::vector<TrackState> tracks_;
  int32_t next_id_{0};
  bool have_stamp_{false};
  double last_stamp_sec_{0.0};
  std::size_t reset_count_{0};
};

}  // namespace people_detector
