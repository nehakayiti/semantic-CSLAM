#pragma once

#include <fstream>
#include <iomanip>
#include <mutex>
#include <string>

#include <definitions.h>
#include <ros/time.h>

namespace sloam {

class InterRobotLCCandidateEvalLogger {
 public:
  InterRobotLCCandidateEvalLogger() = default;

  explicit InterRobotLCCandidateEvalLogger(const std::string& path) {
    setPath(path);
  }

  void setPath(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex_);
    path_ = path;
    header_written_ = false;
    initFileIfNeededUnlocked();
  }

  void logCandidateDecision(int event_id,
                            const ros::Time& stamp,
                            int hostRobotID,
                            int targetRobotID,
                            const SE3& TF_target_to_host,
                            int repeatCount,
                            const std::string& status,
                            size_t reference_map_size,
                            size_t query_map_size) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (path_.empty()) {
      return;
    }
    initFileIfNeededUnlocked();

    const auto q = TF_target_to_host.so3().unit_quaternion();

    std::ofstream out(path_, std::ios::app);
    out << std::fixed << std::setprecision(9)
        << event_id << ","
        << stamp.toSec() << ","
        << hostRobotID << ","
        << targetRobotID << ","
        << status << ","
        << repeatCount << ","
        << reference_map_size << ","
        << query_map_size << ","
        << TF_target_to_host.translation().x() << ","
        << TF_target_to_host.translation().y() << ","
        << TF_target_to_host.translation().z() << ","
        << q.x() << ","
        << q.y() << ","
        << q.z() << ","
        << q.w()
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
          << "timestamp_sec,"
          << "host_robot_id,"
          << "target_robot_id,"
          << "status,"
          << "repeat_count,"
          << "reference_map_size,"
          << "query_map_size,"
          << "candidate_tx,"
          << "candidate_ty,"
          << "candidate_tz,"
          << "candidate_qx,"
          << "candidate_qy,"
          << "candidate_qz,"
          << "candidate_qw\n";
    }

    header_written_ = true;
  }

  std::string path_;
  bool header_written_ = false;
  std::mutex mutex_;
};

}  // namespace sloam