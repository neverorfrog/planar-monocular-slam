#include "pms/optimization/bundle_adjuster.h"

#include <cassert>
#include <cstddef>

#include "pms/dataset/landmark.h"
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

    // Initialize pose measurements from the trajectory
    pose_measurements.reserve(dataset.trajectory.size() - 1);
    for (size_t i = 0; i < dataset.trajectory.size() - 1; ++i) {
        const Pose2& pose_i = dataset.trajectory[i].odometry.getPose2();
        const Pose2& pose_j = dataset.trajectory[i + 1].odometry.getPose2();
        const Pose2 relative_pose = pose_i.inverse() * pose_j;
        pose_measurements.push_back(relative_pose);
    }
}

const BundleAdjuster::OptimizationStats& BundleAdjuster::performIteration() {
    Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> H
        = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(state.getDimension(),
                                                                      state.getDimension());
    Eigen::Matrix<Scalar, Eigen::Dynamic, 1> b
        = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>::Zero(state.getDimension());

    // 1. For each constraint
    // 1.1 Compute projection error and jacobian for each constraint pair
    // 1.2 Robust Kernel to determine inliers
    // 1.3 Put Jacobian into H

    // Pose-Landmark Constraints
    int num_inliers = 0;
    Scalar landmark_chi = 0.0;
    for (const TrajPoint& traj_point : dataset.trajectory) {
        for (const Measurement& meas : traj_point.measurements) {
            const Pose2& pose = state.robot_poses[meas.pose_id];
            const Landmark& landmark = state.landmarks[meas.landmark_id];
            const PoseLandmarkConstraint constraint(pose, landmark.position, meas.image_point, camera);

            landmark_chi += constraint.chi;
            num_inliers += constraint.is_inlier ? 1 : 0;

            // Get the indices for the pose and landmark in the state vector
            constexpr int pose_dim = 3;  // Pose has 3 parameters (x, y, theta)
            constexpr int lm_dim = 3;    // Landmark has 3 parameters (x, y, z)
            int pose_index = meas.pose_id * pose_dim;
            int lm_index = state.getNumPoses() * pose_dim + meas.landmark_id * lm_dim;

            // Fill the Hessian and gradient vector
            H.block<pose_dim, pose_dim>(pose_index, pose_index)
                += constraint.Jr.transpose() * constraint.omega * constraint.Jr;
            H.block<lm_dim, pose_dim>(lm_index, pose_index)
                += constraint.Jl.transpose() * constraint.omega * constraint.Jr;
            H.block<lm_dim, lm_dim>(lm_index, lm_index)
                += constraint.Jl.transpose() * constraint.omega * constraint.Jl;
            H.block<pose_dim, lm_dim>(pose_index, lm_index)
                += constraint.Jr.transpose() * constraint.omega * constraint.Jl;
            b.segment<pose_dim>(pose_index)
                += constraint.Jr.transpose() * constraint.omega * constraint.error;
            b.segment<lm_dim>(lm_index)
                += constraint.Jl.transpose() * constraint.omega * constraint.error;
        }
    }
    stats.landmark_chi = landmark_chi;

    // Pose-Pose Constraints
    if (config.pose_pose) {
        Scalar pose_chi = 0.0;
        for (size_t i = 0; i < state.getNumPoses() - 1; ++i) {
            // Construct the constraint for the consecutive poses
            const Pose2& pose_i = state.robot_poses[i];
            const Pose2& pose_j = state.robot_poses[i + 1];
            const PosePoseConstraint constraint(pose_i, pose_j, pose_measurements[i]);
            pose_chi += constraint.chi;

            // Get the indices for the poses in the state vector
            constexpr int pose_dim = Pose2Manifold::getDimension();  // Pose has 3 parameters (x, y, theta)
            int pose_i_index = i * pose_dim;
            int pose_j_index = (i + 1) * pose_dim;

            // Fill the Hessian and gradient vector
            H.block<pose_dim, pose_dim>(pose_i_index, pose_i_index)
                += constraint.Ji.transpose() * constraint.omega * constraint.Ji;
            H.block<pose_dim, pose_dim>(pose_j_index, pose_j_index)
                += constraint.Jj.transpose() * constraint.omega * constraint.Jj;
            H.block<pose_dim, pose_dim>(pose_i_index, pose_j_index)
                += constraint.Ji.transpose() * constraint.omega * constraint.Jj;
            H.block<pose_dim, pose_dim>(pose_j_index, pose_i_index)
                += constraint.Jj.transpose() * constraint.omega * constraint.Ji;
            b.segment<pose_dim>(pose_i_index)
                += constraint.Ji.transpose() * constraint.omega * constraint.error;
            b.segment<pose_dim>(pose_j_index)
                += constraint.Jj.transpose() * constraint.omega * constraint.error;
        }
        stats.pose_chi = pose_chi;
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
    stats.num_inliers = num_inliers;
    Scalar old_position_error = stats.position_error;
    stats.position_error = 0.0;
    stats.orientation_error = 0.0;
    stats.map_error = 0.0;

    for (size_t i = 0; i < state.getNumPoses() - 1; ++i) {
        const Pose2& pose_current = state.robot_poses[i];
        const Pose2& pose_next = state.robot_poses[i + 1];
        Pose2 estimated_relative_pose = pose_current.inverse() * pose_next;
        const Pose2& gt_pose_current = dataset.trajectory[i].ground_truth.getPose2();
        const Pose2& gt_pose_next = dataset.trajectory[i + 1].ground_truth.getPose2();
        Pose2 gt_relative_pose = gt_pose_current.inverse() * gt_pose_next;
        Pose2 error_pose = estimated_relative_pose.inverse() * gt_relative_pose;

        stats.position_error += std::sqrt(
            (std::pow(error_pose.translation.x(), 2) + std::pow(error_pose.translation.y(), 2)) / 2);
        stats.orientation_error += std::abs(error_pose.rotation);
    }

    for (size_t i = 0; i < state.getNumLandmarks(); ++i) {
        const Landmark& estimated_lm = state.landmarks[i];
        const Landmark& gt_lm = dataset.world[i];
        if (estimated_lm.valid) {
            Vector3 diff = estimated_lm.position - gt_lm.position;
            stats.map_error
                += std::sqrt((std::pow(diff.x(), 2) + std::pow(diff.y(), 2) + std::pow(diff.z(), 2)) / 3);
        }
    }
    stats.map_error /= state.getNumLandmarks();

    if (stats.position_error < config.tolerance && stats.orientation_error < config.tolerance) {
        stats.converged = true;
    } else {
        stats.converged = false;
    }

    if (std::abs(stats.position_error - old_position_error) < config.min_delta) {
        patience_counter++;
        stats.converged = patience_counter >= config.patience;
    } else {
        patience_counter = 0;
    }

    return stats;
}

}  // namespace pms
