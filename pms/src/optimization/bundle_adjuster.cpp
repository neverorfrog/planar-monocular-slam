#include "pms/optimization/bundle_adjuster.h"

#include <cassert>
#include <cstddef>
#include <iostream>

#include "pms/dataset/trajectory_point.h"
#include "pms/math/pose2.h"
#include "pms/optimization/manifold.h"
#include "pms/optimization/pose_landmark_constraint.h"
#include "pms/optimization/pose_pose_constraint.h"

namespace pms {

BundleAdjuster::BundleAdjuster(const std::vector<Landmark>& landmarks, const Dataset& dataset,
                               const BundleAdjustmentConfig& config)
    : config(config), stats{}, dataset(dataset) {
    state = State(dataset.getOdometryPoses2(), landmarks);
    camera = dataset.camera;
    std::cout << "NUMBER OF ODOM POSES " << state.getNumPoses() << std::endl;
    std::cout << "NUMBER OF LANDMARKS " << state.getNumLandmarks() << std::endl;
}

const BundleAdjuster::OptimizationStats& BundleAdjuster::performIteration() {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> H
        = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(state.getDimension(),
                                                                      state.getDimension());
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> b
        = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(state.getDimension());

    std::cout << "Starting bundle adjustment iteration..." << std::endl;

    // 1. For each constraint
    // 1.1 Compute projection error and jacobian for each constraint pair
    // 1.2 Robust Kernel to determine inliers
    // 1.3 Put Jacobian into H

    // Pose-Landmark Constraints
    Scalar landmark_chi = 0.0;
    for (const TrajPoint& traj_point : dataset.trajectory) {
        for (const Measurement& meas : traj_point.measurements) {
            const Pose2& pose = state.robot_poses[meas.pose_id];
            const Landmark& landmark = state.landmarks[meas.landmark_id];

            // if (!landmark.valid) {
            //     continue;
            // }

            const PoseLandmarkConstraint constraint(pose, landmark.position, meas.image_point, camera);

            if (!constraint.Jr.allFinite() || !constraint.Jl.allFinite()) {
                continue;
            }

            landmark_chi += constraint.chi;

            // Get the indices for the pose and landmark in the state vector
            constexpr int pose_dim = Pose2Manifold::getDimension();  // Pose has 3 parameters (x, y, theta)
            constexpr int lm_dim = 3;                                // Landmark has 3 parameters (x, y, z)
            int pose_index = meas.pose_id * pose_dim;
            int lm_index = state.getNumPoses() * pose_dim + meas.landmark_id * lm_dim;
            assert(pose_index >= 0 && pose_index + pose_dim <= state.getDimension());
            assert(lm_index >= 0 && lm_index + lm_dim <= state.getDimension());

            // Fill the Hessian and gradient vector
            H.block<pose_dim, pose_dim>(pose_index, pose_index) += constraint.Jr.transpose() * constraint.Jr;
            H.block<lm_dim, pose_dim>(lm_index, pose_index) += constraint.Jl.transpose() * constraint.Jr;
            H.block<lm_dim, lm_dim>(lm_index, lm_index) += constraint.Jl.transpose() * constraint.Jl;
            H.block<pose_dim, lm_dim>(pose_index, lm_index) += constraint.Jr.transpose() * constraint.Jl;
            b.segment<pose_dim>(pose_index) += constraint.Jr.transpose() * constraint.error;
            b.segment<lm_dim>(lm_index) += constraint.Jl.transpose() * constraint.error;
        }
    }

    // Pose-Pose Constraints
    Scalar pose_chi = 0.0;
    for (size_t i = 0; i < state.getNumPoses() - 1; ++i) {
        // Construct the constraint for the consecutive poses
        const Pose2& pose_i = state.robot_poses[i];
        const Pose2& pose_j = state.robot_poses[i + 1];
        const PosePoseConstraint constraint(pose_i, pose_j);

        // Get the indices for the poses in the state vector
        constexpr int pose_dim = Pose2Manifold::getDimension();  // Pose has 3 parameters (x, y, theta)
        int pose_i_index = i * pose_dim;
        int pose_j_index = (i + 1) * pose_dim;

        // Fill the Hessian and gradient vector
        H.block<pose_dim, pose_dim>(pose_i_index, pose_i_index) += constraint.Ji.transpose() * constraint.Ji;
        H.block<pose_dim, pose_dim>(pose_j_index, pose_j_index) += constraint.Jj.transpose() * constraint.Jj;
        H.block<pose_dim, pose_dim>(pose_i_index, pose_j_index) += constraint.Ji.transpose() * constraint.Jj;
        H.block<pose_dim, pose_dim>(pose_j_index, pose_i_index) += constraint.Jj.transpose() * constraint.Ji;
        b.segment<pose_dim>(pose_i_index) += constraint.Ji.transpose() * constraint.error;
        b.segment<pose_dim>(pose_j_index) += constraint.Jj.transpose() * constraint.error;
        pose_chi += constraint.error.transpose() * constraint.error;
    }

    // 2. Solve the linear system H * delta = -b
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> hessian
        = H.block(3, 3, H.rows() - 3, H.cols() - 3);  // Exclude the first 3 dimensions (robot pose)
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> gradient
        = b.segment(3, b.size() - 3);  // Exclude the first 3 dimensions (robot pose)

    VectorX delta = VectorX::Zero(H.cols());
    delta.segment(3, hessian.cols()) = -hessian.ldlt().solve(gradient);
    state.applyIncrement(delta);

    // 3. Update stats with actual values
    stats.num_iterations++;
    stats.pose_chi = pose_chi;
    stats.landmark_chi = landmark_chi;
    stats.converged = (landmark_chi < config.tolerance
                       && pose_chi < config.tolerance);  // TODO: is this correct? And enough?
    return stats;
}

}  // namespace pms
