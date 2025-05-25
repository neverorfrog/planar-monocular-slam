#pragma once

#include <cassert>
#include <iostream>

#include "Eigen/Core"
#include "pms/dataset.h"
#include "pms/math/definitions.h"
#include "pms/solution.h"

namespace pms {

inline std::pair<Vector3, bool> triangulateLandmark(const std::vector<Pose3>& odom_poses,
                                                    const std::vector<Measurement>& measurements,
                                                    const Camera& camera) {
    /**
     * Triangulate the 3D position of a landmark given its 2D measurements,
     * each associated with a robot pose, and the camera intrinsics. This
     * function should return the guessed 3D position of the landmark.
     */

    const int M = odom_poses.size();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 4> A = Eigen::Matrix<Scalar, Eigen::Dynamic, 4>::Zero(2 * M, 4);

    if (A.rows() < A.cols()) {
        std::cerr << "Not enough measurements to triangulate landmark." << std::endl;
        return std::make_pair(Vector3::Zero(), false);
    }

    std::vector<Pose3> camera_T_world_frames = std::vector<Pose3>(M);

    for (int i = 0; i < M; ++i) {
        // Compute projection matrix
        const Pose3& world_T_robot = odom_poses[i];  // transforms from robot frame to world frame
        const Pose3& robot_T_camera = camera.pose;   // transforms from camera frame to robot frame
        const Pose3 world_T_camera = (world_T_robot * robot_T_camera);
        const Pose3 camera_T_world = world_T_camera.inverse();  // transforms from world frame to camera frame
        camera_T_world_frames.at(i) = camera_T_world;
        Eigen::Matrix<Scalar, 3, 4> projection = Eigen::Matrix<Scalar, 3, 4>::Zero();
        projection.block(0, 0, 3, 3) = camera_T_world.rotation;
        projection.block(0, 3, 3, 1) = camera_T_world.translation;
        projection = camera.camera_matrix * projection;

        // Fill coefficient matrix for homogeneus system
        const Measurement& measurement = measurements[i];
        Scalar u = measurement.image_point(0);
        Scalar v = measurement.image_point(1);
        A.row(i * 2) = v * projection.row(2) - projection.row(1);
        A.row(i * 2 + 1) = projection.row(0) - u * projection.row(2);
    }

    Eigen::JacobiSVD<decltype(A)> svd(A, Eigen::ComputeThinV);
    Eigen::Matrix<Scalar, 4, 1> X = svd.matrixV().rightCols(1);
    if (std::abs(X(3)) < 1e-9) {
        std::cerr << "Triangulation failed: homogeneous coordinate w is near zero." << std::endl;
        return std::make_pair(Vector3::Zero(), false);
    }
    Vector3 p = X.head<3>() / X(3, 0);  // Dehomogenize

    // Cheirality check
    for (int i = 0; i < M; ++i) {
        Eigen::Matrix<Scalar, 4, 1> p_hom;
        p_hom.head<3>() = p;
        p_hom(3) = 1.0;
        Vector4 X_cam = camera_T_world_frames.at(i).getHomogen() * p_hom;
        Scalar depth_in_camera = X_cam(2) / X_cam(3);
        if (depth_in_camera <= 0) {
            std::cerr << "Cheirality check failed: Landmark "
                      << " behind or on camera plane for view " << i << ". Depth: " << depth_in_camera
                      << std::endl;
            return std::make_pair(Vector3::Zero(), false);
        }

        if (depth_in_camera > camera.z_far) {
            std::cerr << "Cheirality/Range check failed: Landmark beyond z_far for view "
                      << i << ". Depth: " << depth_in_camera << " > z_far: " << camera.z_far << std::endl;
            return std::make_pair(Vector3::Zero(), false);
        }
    }

    return std::make_pair(p, true);
}

inline void triangulate(Solution& solution, const Dataset& dataset) {
    // Parse measurement for each landmark
    // TODO: Should i consider only measurements where the landmark is also visible in the next or previous
    // frame?
    std::vector<std::vector<Measurement>> measurements_per_landmark(dataset.world.size());
    for (const auto& traj_point : dataset.trajectory) {
        for (const auto& measurement : traj_point.measurements) {
            int landmark_id = measurement.landmark_id;
            assert(landmark_id >= 0 && landmark_id < dataset.world.size());
            measurements_per_landmark[landmark_id].push_back(measurement);
        }
    }

    // Get odometry poses for each measurement in each landmark and triangulate
    int failed_guesses = 0;
    for (size_t i = 0; i < measurements_per_landmark.size(); ++i) {
        std::cout << "Triangulating landmark " << i << " with " << measurements_per_landmark[i].size()
                  << " measurements." << std::endl;
        const std::vector<Measurement>& measurements = measurements_per_landmark[i];
        if (measurements.size() < 2) {
            std::cerr << "Not enough measurements to triangulate landmark " << i << "\n\n";
            continue;
        }

        std::vector<Pose3> odom_poses;
        for (const auto& measurement : measurements) {
            int seq_number = measurement.seq_number;
            assert(seq_number >= 0 && seq_number < dataset.trajectory.size());
            odom_poses.push_back(dataset.trajectory[seq_number].odometry);
        }

        std::pair<Vector3, bool> landmark_pos_guess
            = triangulateLandmark(odom_poses, measurements, dataset.camera);
        failed_guesses += !landmark_pos_guess.second ? 1 : 0;
        solution.world.at(i).position = landmark_pos_guess.first;
        solution.world.at(i).valid = landmark_pos_guess.second;
    }

    std::cout << "FAILED GUESSES " << failed_guesses << "\n\n";
}

}  // namespace pms
