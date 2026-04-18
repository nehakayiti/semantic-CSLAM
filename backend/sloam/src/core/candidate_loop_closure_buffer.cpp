#include "core/candidate_loop_closure_buffer.h"

#include <algorithm>
#include <cmath>

namespace sloam {

CandidateLoopClosureBuffer::CandidateLoopClosureBuffer(const Params& params)
    : params_(params) {
}

std::optional<AcceptedLoopClosure> CandidateLoopClosureBuffer::addCandidate(const LoopClosureCandidate& candidate) {
    // Always keep the candidate in the rolling buffer.
    buffer.push_back(candidate);

    while (buffer.size() > params_.max_candidates) {
        buffer.pop_front();
    }

    // // First check basic quality on the candidate itself.
    // if (!isQualityGood(candidate)) {
    //     return std::nullopt;
    // }

    // Count how many recent candidates are consistent with this one.
    int repeat_count = countConsistentMatches(candidate);

    if (repeat_count < params_.repeat_count_threshold) {
        return std::nullopt;
    }

    // Build accepted loop closure result.
    AcceptedLoopClosure accepted;
    accepted.hostRobotID = candidate.hostRobotID;
    accepted.targetRobotID = candidate.targetRobotID;
    //accepted.hostPoseIdx = candidate.hostPoseIdx;
    //accepted.targetPoseIdx = candidate.targetPoseIdx;
    accepted.TF_target_to_host = candidate.TF_target_to_host;
    accepted.repeatCount = repeat_count;
    //accepted.inlierCount = candidate.inlierCount;
    //accepted.residual = candidate.residual;

    return accepted;
}

void CandidateLoopClosureBuffer::clear() {
    buffer.clear();
}

// bool CandidateLoopClosureBuffer::isQualityGood(const LoopClosureCandidate& candidate) const {
//     if (candidate.inlierCount < params_.minimum_inliers) {
//         return false;
//     }

//     if (candidate.residual > params_.maximum_residual) {
//         return false;
//     }

//     return true;
// }

bool CandidateLoopClosureBuffer::isSameRobotPair(const LoopClosureCandidate& a, const LoopClosureCandidate& b) const {
    return (a.hostRobotID == b.hostRobotID) &&
           (a.targetRobotID == b.targetRobotID);
}

// bool CandidateLoopClosureBuffer::isNearbyPosePair(const LoopClosureCandidate& a, const LoopClosureCandidate& b) const {
//     return (std::abs(a.hostPoseIdx - b.hostPoseIdx) <= params_.max_pose_index_diff) &&
//            (std::abs(a.targetPoseIdx - b.targetPoseIdx) <= params_.max_pose_index_diff);
// }

bool CandidateLoopClosureBuffer::isTransformConsistent(const LoopClosureCandidate& a, const LoopClosureCandidate& b) const {
    double translation_diff = translationDifference(a.TF_target_to_host, b.TF_target_to_host);
    double rotation_diff_deg = rotationDifferenceDeg(a.TF_target_to_host, b.TF_target_to_host);

    if (translation_diff > params_.max_translation_diff) {
        return false;
    }

    if (rotation_diff_deg > params_.max_rotation_diff_deg) {
        return false;
    }

    return true;
}

double CandidateLoopClosureBuffer::translationDifference(const SE3& a, const SE3& b) const {
    return (a.translation() - b.translation()).norm();
}

double CandidateLoopClosureBuffer::rotationDifferenceDeg(const SE3& a, const SE3& b) const {
    Eigen::Matrix3d R_rel = a.so3().matrix().transpose() * b.so3().matrix();
    double trace_val = R_rel.trace();
    double cos_angle = (trace_val - 1.0) / 2.0;

    cos_angle = std::max(-1.0, std::min(1.0, cos_angle));

    double angle_rad = std::acos(cos_angle);
    return angle_rad * 180.0 / M_PI;
}

int CandidateLoopClosureBuffer::countConsistentMatches(const LoopClosureCandidate& candidate) const {
    int count = 0;

    for (const auto& old_candidate : buffer) {
        if (!isSameRobotPair(candidate, old_candidate)) {
            continue;
        }

        // if (!isNearbyPosePair(candidate, old_candidate)) {
        //     continue;
        // }

        // if (!isQualityGood(old_candidate)) {
        //     continue;
        // }

        if (!isTransformConsistent(candidate, old_candidate)) {
            continue;
        }

        count++;
    }

    return count;
}

} // namespace sloam