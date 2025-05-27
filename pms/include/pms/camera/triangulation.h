#pragma once

#include <cassert>
#include <iostream>

#include "Eigen/Core"
#include "pms/dataset/dataset.h"
#include "pms/dataset/landmark.h"
#include "pms/math/definitions.h"

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
    Pose3 camera_T_world;
    Eigen::Matrix<Scalar, 3, 4> projection;

    for (int i = 0; i < M; ++i) {
        // Compute projection matrix
        camera_T_world = camera.computeWorldToCamera(odom_poses[i]);
        camera_T_world_frames.at(i) = camera_T_world;
        projection = camera.computeProjectionMatrix(camera_T_world);

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
            std::cerr << "Cheirality/Range check failed: Landmark beyond z_far for view " << i
                      << ". Depth: " << depth_in_camera << " > z_far: " << camera.z_far << std::endl;
            return std::make_pair(Vector3::Zero(), false);
        }
    }

    return std::make_pair(p, true);
}

inline std::vector<Landmark> triangulate(const Dataset& dataset) {
    std::vector<Landmark> guessed_landmarks;

    // TODO: Should i consider only meas where the landmark is also visible in the next or previous frame?
    std::vector<std::vector<Measurement>> measurements_per_landmark = dataset.getMeasurementsPerLandmark();

    // Get odometry poses for each measurement in each landmark and triangulate
    for (size_t i = 0; i < measurements_per_landmark.size(); ++i) {
        const std::vector<Measurement>& measurements = measurements_per_landmark[i];
        if (measurements.size() < 2) {
            std::cerr << "Not enough measurements to triangulate landmark " << i << "\n\n";
            guessed_landmarks.push_back(Landmark(Vector3::Zero(), i, false));
            continue;
        }

        std::vector<Pose3> odom_poses;
        for (const auto& measurement : measurements) {
            int pose_id = measurement.pose_id;
            assert(pose_id >= 0 && pose_id < dataset.trajectory.size());
            odom_poses.push_back(dataset.trajectory[pose_id].odometry);
        }

        std::pair<Vector3, bool> landmark_pos_guess
            = triangulateLandmark(odom_poses, measurements, dataset.camera);
        guessed_landmarks.push_back(Landmark(landmark_pos_guess.first, i, landmark_pos_guess.second));
    }

    return guessed_landmarks;
}

}  // namespace pms
