#pragma once

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>
#include <vector>

#include <definitions.h>
#include <ros/time.h>

namespace sloam {

struct TrajectoryJumpStats {
  double max_translation_jump_m = 0.0;
  double max_rotation_jump_deg = 0.0;
  size_t compared_pose_count = 0;
};

struct PendingInterRobotStabilityEvalEvent {
  int event_id = -1;
  int hostRobotID = -1;
  int targetRobotID = -1;
  SE3 TF_target_to_host;
  int repeatCount = 0;
  ros::Time accepted_stamp = ros::Time(0);
  size_t reference_map_size = 0;
  size_t query_map_size = 0;
};

inline double interRobotStabilityEvalRotationDifferenceDeg(const SE3& a,
                                                           const SE3& b) {
  const Eigen::Matrix3d R_rel = a.so3().matrix().transpose() * b.so3().matrix();
  double cos_angle = (R_rel.trace() - 1.0) / 2.0;
  cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
  const double angle_rad = std::acos(cos_angle);
  return angle_rad * 180.0 / 3.14159265358979323846;
}

inline TrajectoryJumpStats computeTrajectoryJumpStats(
    const std::vector<SE3>& before,
    const std::vector<SE3>& after) {
  TrajectoryJumpStats stats;
  const size_t n = std::min(before.size(), after.size());
  stats.compared_pose_count = n;

  for (size_t i = 0; i < n; ++i) {
    const double translation_jump =
        (before[i].translation() - after[i].translation()).norm();
    const double rotation_jump_deg =
        interRobotStabilityEvalRotationDifferenceDeg(before[i], after[i]);

    if (translation_jump > stats.max_translation_jump_m) {
      stats.max_translation_jump_m = translation_jump;
    }
    if (rotation_jump_deg > stats.max_rotation_jump_deg) {
      stats.max_rotation_jump_deg = rotation_jump_deg;
    }
  }

  return stats;
}

class InterRobotLCStabilityEvalLogger {
 public:
  InterRobotLCStabilityEvalLogger() = default;

  explicit InterRobotLCStabilityEvalLogger(const std::string& path) {
    setPath(path);
  }

  void setPath(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = path;
    header_written_ = false;
    initFileIfNeededUnlocked();
  }

  void logAcceptedSolve(const PendingInterRobotStabilityEvalEvent& event,
                        const TrajectoryJumpStats& host_jump_stats,
                        const TrajectoryJumpStats& target_jump_stats) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path_.empty()) {
      return;
    }
    initFileIfNeededUnlocked();

    const auto q = event.TF_target_to_host.so3().unit_quaternion();

    const double combined_max_translation_jump_m = std::max(
        host_jump_stats.max_translation_jump_m,
        target_jump_stats.max_translation_jump_m);

    const double combined_max_rotation_jump_deg = std::max(
        host_jump_stats.max_rotation_jump_deg,
        target_jump_stats.max_rotation_jump_deg);

    std::ofstream out(path_, std::ios::app);
    out << std::fixed << std::setprecision(9)
        << event.event_id << ","
        << event.accepted_stamp.toSec() << ","
        << event.hostRobotID << ","
        << event.targetRobotID << ","
        << event.repeatCount << ","
        << event.reference_map_size << ","
        << event.query_map_size << ","
        << event.TF_target_to_host.translation().x() << ","
        << event.TF_target_to_host.translation().y() << ","
        << event.TF_target_to_host.translation().z() << ","
        << q.x() << ","
        << q.y() << ","
        << q.z() << ","
        << q.w() << ","
        << host_jump_stats.max_translation_jump_m << ","
        << host_jump_stats.max_rotation_jump_deg << ","
        << target_jump_stats.max_translation_jump_m << ","
        << target_jump_stats.max_rotation_jump_deg << ","
        << combined_max_translation_jump_m << ","
        << combined_max_rotation_jump_deg << ","
        << host_jump_stats.compared_pose_count << ","
        << target_jump_stats.compared_pose_count
        << "\n";
  }

 private:
  void initFileIfNeededUnlocked() {
    if (path_.empty() || header_written_) {
      return;
    }

    bool file_has_content = false;
    {
      std::ifstream in(path_);
      file_has_content =
          in.good() && (in.peek() != std::ifstream::traits_type::eof());
    }

    if (!file_has_content) {
      std::ofstream out(path_);
      out << "event_id,"
          << "accepted_timestamp_sec,"
          << "host_robot_id,"
          << "target_robot_id,"
          << "repeat_count,"
          << "reference_map_size,"
          << "query_map_size,"
          << "candidate_tx,"
          << "candidate_ty,"
          << "candidate_tz,"
          << "candidate_qx,"
          << "candidate_qy,"
          << "candidate_qz,"
          << "candidate_qw,"
          << "host_max_translation_jump_m,"
          << "host_max_rotation_jump_deg,"
          << "target_max_translation_jump_m,"
          << "target_max_rotation_jump_deg,"
          << "combined_max_translation_jump_m,"
          << "combined_max_rotation_jump_deg,"
          << "host_compared_pose_count,"
          << "target_compared_pose_count\n";
    }

    header_written_ = true;
  }

  std::string path_;
  bool header_written_ = false;
  std::mutex mutex_;
};

}  // namespace sloam