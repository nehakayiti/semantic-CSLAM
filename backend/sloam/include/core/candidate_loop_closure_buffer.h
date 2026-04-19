#pragma once

#include <deque>
#include <optional>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <definitions.h>

namespace sloam {

struct LoopClosureCandidate {
    SE3 TF_target_to_host; // the estimated transform from target robot to host robot

    int hostRobotID = -1;
    int targetRobotID = -1;

    //int hostPoseIdx = -1;
    //int targetPoseIdx = -1;
    //int inlierCount = 0;
    //float residual = 1e9;
};

struct AcceptedLoopClosure {
    SE3 TF_target_to_host; // the estimated transform from target robot to host robot

    int hostRobotID = -1;
    int targetRobotID = -1;

    //int hostPoseIdx = -1;
    //int targetPoseIdx = -1;

    int repeatCount = 0;
};

struct CandidateLoopClosureDecision {
    bool accepted = false;
    int repeatCount = 0;
    std::optional<AcceptedLoopClosure> acceptedLoopClosure;
};

class CandidateLoopClosureBuffer {
  public:
    struct Params {
        int max_candidates = 10;
        int repeat_count_threshold = 2;
        //int minimum_inliers = 20;
        //double maximum_residual = 1.0;
        double max_translation_diff = 1.0;
        double max_rotation_diff_deg = 15.0;
        //int max_pose_index_diff = 2;
    };

    explicit CandidateLoopClosureBuffer(const Params& params);

    std::optional<AcceptedLoopClosure> addCandidate(const LoopClosureCandidate& candidate);

    CandidateLoopClosureDecision addCandidateWithDecision(
        const LoopClosureCandidate& candidate);

    void clear();

  private:
    Params params_;
    std::deque<LoopClosureCandidate> buffer;

    bool isSameRobotPair(const LoopClosureCandidate& a,
                         const LoopClosureCandidate& b) const;
    bool isTransformConsistent(const LoopClosureCandidate& a,
                               const LoopClosureCandidate& b) const;

    double translationDifference(const SE3& a, const SE3& b) const;
    double rotationDifferenceDeg(const SE3& a, const SE3& b) const;

    int countConsistentMatches(const LoopClosureCandidate& candidate) const;
};

} // namespace sloam