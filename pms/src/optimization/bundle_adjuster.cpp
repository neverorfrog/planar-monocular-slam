#include "pms/optimization/bundle_adjuster.h"

#include <cassert>
#include <cstddef>
#include <iostream>

#include "pms/math/pose2.h"
#include "pms/optimization/constraints.h"
#include "pms/optimization/manifold.h"

namespace pms {

BundleAdjuster::BundleAdjuster(const std::vector<Landmark>& landmarks, const Dataset& dataset,
                               const BundleAdjustmentConfig& config)
    : config(config), stats{} {
    state = State(dataset.getOdometryPoses2(), landmarks);
    camera = dataset.camera;
    measurements = dataset.getFlatMeasurements(landmarks);
    std::cout << "NUMBER OF LANDMARKS: " << measurements.size() << std::endl;
    std::cout << "BundleAdjuster initialized with max iterations: " << config.max_iterations
              << " and tolerance: " << config.tolerance << std::endl;
    std::cout << "NUMBER OF ODOM POSES " << state.getNumPoses() << std::endl;
    std::cout << "NUMBER OF LANDMARKS " << state.getNumLandmarks() << std::endl;

    int valid_index = 0;
    for (size_t i = 0; i < state.landmarks.size(); ++i) {
        if (state.landmarks[i].valid) {
            valid_landmarks.push_back(valid_index);
            valid_index++;
        } else {
            valid_landmarks.push_back(-1);
        }
    }
    assert(valid_index == state.getNumLandmarks()
           && "Number of valid landmarks does not match the number of landmarks in the state");
}

bool BundleAdjuster::performIteration() {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> H
        = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(state.getStateDimension(),
                                                                      state.getStateDimension());
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> b
        = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(state.getStateDimension());

    // 1. For each constraint
    // 1.1 Compute projection error and jacobian for each constraint pair
    // TODO: Compute kernel
    // 1.3 Put Jacobian into H

    // Pose-Landmark Constraints
    for (size_t i = 0; i < measurements.size(); ++i) {
        // Construct the constraint for the current measurement
        const Measurement& meas = measurements[i];
        const Pose2& pose = state.robot_poses[meas.pose_id];
        const Landmark& landmark = state.landmarks[meas.landmark_id];
        assert(landmark.valid && "Landmark must be valid");
        const PoseLandmarkConstraint constraint(pose, landmark.position, meas.image_point);

        // Get the indices for the pose and landmark in the state vector
        constexpr int pose_dim = Pose2Manifold::getDimension();  // Pose has 3 parameters (x, y, theta)
        constexpr int lm_dim = 3;                                // Landmark has 3 parameters (x, y, z)
        int pose_index = meas.pose_id * pose_dim;
        int lm_index = state.getNumPoses() * pose_dim + getValidLandmarkIndex(meas.landmark_id) * lm_dim;
        assert(pose_index >= 0 && pose_index + pose_dim <= state.getStateDimension());
        assert(lm_index >= 0 && lm_index + lm_dim <= state.getStateDimension());

        // Fill the Hessian and gradient vector
        H.block<pose_dim, pose_dim>(pose_index, pose_index) += constraint.Jr.transpose() * constraint.Jr;
        H.block<lm_dim, pose_dim>(lm_index, pose_index) += constraint.Jl.transpose() * constraint.Jr;
        H.block<lm_dim, lm_dim>(lm_index, lm_index) += constraint.Jl.transpose() * constraint.Jl;
        H.block<pose_dim, lm_dim>(pose_index, lm_index) += constraint.Jr.transpose() * constraint.Jl;
        b.segment<pose_dim>(pose_index) += constraint.Jr.transpose() * constraint.error;
        b.segment<lm_dim>(lm_index) += constraint.Jl.transpose() * constraint.error;
    }

    // Pose-Pose Constraints
    for (size_t i = 0; i < state.getNumPoses() - 1; ++i) {
        // Construct the constraint for the consecutive poses
        const Pose2& pose_i = state.robot_poses[i];
        const Pose2& pose_j = state.robot_poses[i + 1];
        const PosePoseConstraint constraint(pose_i, pose_j);

        // Get the indices for the poses in the state vector
        constexpr int pose_dim = Pose2Manifold::getDimension();  // Pose has 3 parameters (x, y, theta)
        int pose_i_index = i * pose_dim;
        int pose_j_index = (i + 1) * pose_dim;
        assert(pose_index >= 0 && pose_index + pose_dim <= state.getStateDimension());
        assert(lm_index >= 0 && lm_index + lm_dim <= state.getStateDimension());

        // Fill the Hessian and gradient vector
        H.block<pose_dim, pose_dim>(pose_i_index, pose_i_index) += constraint.Ji.transpose() * constraint.Ji;
        H.block<pose_dim, pose_dim>(pose_j_index, pose_j_index) += constraint.Jj.transpose() * constraint.Jj;
        H.block<pose_dim, pose_dim>(pose_i_index, pose_j_index) += constraint.Ji.transpose() * constraint.Jj;
        H.block<pose_dim, pose_dim>(pose_j_index, pose_i_index) += constraint.Jj.transpose() * constraint.Ji;
        b.segment<pose_dim>(pose_i_index) += constraint.Ji.transpose() * constraint.error;
        b.segment<pose_dim>(pose_j_index) += constraint.Jj.transpose() * constraint.error;
    }

    std::cout << "H AND B DONE" << std::endl;

    // 2. Solve the linear system H * delta = -b
    VectorX delta = H.ldlt().solve(-b);
    state.applyIncrement(delta);

    // TODO: Update stats with actual values
    stats.num_iterations++;
    return false;
}

int BundleAdjuster::getValidLandmarkIndex(int landmark_id) const {
    return (landmark_id >= 0) ?
               valid_landmarks[landmark_id] :
               throw std::out_of_range("Invalid landmark index: " + std::to_string(landmark_id));
}

const BundleAdjuster::OptimizationStats& BundleAdjuster::getStats() const {
    return stats;
}

}  // namespace pms
