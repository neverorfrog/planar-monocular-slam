#include "pms/camera/triangulation.h"

#include <cassert>
#include <cmath>

namespace pms {

Triangulator::Triangulator(const Camera& camera, const TriangulationConfig& config)
    : camera_(camera), config_(config) {}

std::pair<Vector3, bool> Triangulator::triangulateLandmark(const std::vector<Pose3>& odom_poses,
                                                         const std::vector<Measurement>& measurements) const {
    // Check minimum observations requirement
    if (measurements.size() < config_.min_observations) {
        return std::make_pair(Vector3::Zero(), false);
    }

    const int M = odom_poses.size();
    Eigen::Matrix<Scalar, Eigen::Dynamic, 4> A = Eigen::Matrix<Scalar, Eigen::Dynamic, 4>::Zero(2 * M, 4);

    std::vector<Pose3> camera_T_world_frames = std::vector<Pose3>(M);
    Pose3 camera_T_world;
    Eigen::Matrix<Scalar, 3, 4> projection;

    for (int i = 0; i < M; ++i) {
        camera_T_world = camera_.computeWorldToCamera(odom_poses[i]);
        camera_T_world_frames.at(i) = camera_T_world;
        projection = camera_.computeProjectionMatrix(camera_T_world);

        const Measurement& measurement = measurements[i];
        Scalar u = measurement.image_point(0);
        Scalar v = measurement.image_point(1);
        
        A.row(i * 2) = v * projection.row(2) - projection.row(1);
        A.row(i * 2 + 1) = projection.row(0) - u * projection.row(2);
    }

    Eigen::JacobiSVD<decltype(A)> svd(A, Eigen::ComputeThinV);
    Eigen::Matrix<Scalar, 4, 1> X = svd.matrixV().rightCols(1);
    
    if (std::abs(X(3)) < config_.homogeneous_threshold) {
        return std::make_pair(Vector3::Zero(), false);
    }
    
    // Dehomogenize to get 3D Cartesian coordinates
    Vector3 p = X.head<3>() / X(3, 0);

    // Cheirality check: ensure the point is in front of all cameras
    if (config_.enable_cheirality_check) {
        for (int i = 0; i < M; ++i) {
            Eigen::Matrix<Scalar, 4, 1> p_hom;
            p_hom.head<3>() = p;
            p_hom(3) = 1.0;
            Vector4 X_cam = camera_T_world_frames.at(i).getHomogen() * p_hom;
            Scalar depth_in_camera = X_cam(2) / X_cam(3);
            
            // Check if point is behind camera or beyond far plane
            if (depth_in_camera <= 0 || depth_in_camera > camera_.z_far) {
                return std::make_pair(Vector3::Zero(), false);
            }
        }
    }

    return std::make_pair(p, true);
}

std::vector<Landmark> Triangulator::triangulateAll(const std::vector<std::vector<Measurement>>& measurements_per_landmark,
                                                  const std::vector<Pose3>& odom_poses) const {
    std::vector<Landmark> triangulated_landmarks;
    triangulated_landmarks.reserve(measurements_per_landmark.size());

    for (size_t i = 0; i < measurements_per_landmark.size(); ++i) {
        const std::vector<Measurement>& measurements = measurements_per_landmark[i];
        
        // Check if we have enough observations
        if (measurements.size() < config_.min_observations) {
            triangulated_landmarks.push_back(Landmark(Vector3::Zero(), i, false));
            continue;
        }

        // Collect corresponding robot poses for each measurement
        std::vector<Pose3> corresponding_poses;
        corresponding_poses.reserve(measurements.size());
        
        for (const auto& measurement : measurements) {
            int pose_id = measurement.pose_id;
            assert(pose_id >= 0 && pose_id < static_cast<int>(odom_poses.size()));
            corresponding_poses.push_back(odom_poses[pose_id]);
        }

        // Triangulate the landmark position
        std::pair<Vector3, bool> result = triangulateLandmark(corresponding_poses, measurements);
        triangulated_landmarks.push_back(Landmark(result.first, i, result.second));
    }

    return triangulated_landmarks;
}

std::vector<Landmark> Triangulator::triangulateFromDataset(const Dataset& dataset) const {
    std::vector<std::vector<Measurement>> measurements_per_landmark = dataset.getMeasurementsPerLandmark();
    std::vector<Pose3> odom_poses = dataset.getOdometryPoses3();
    return triangulateAll(measurements_per_landmark, odom_poses);
}

std::pair<Vector3, bool> triangulateLandmark(const std::vector<Pose3>& odom_poses,
                                           const std::vector<Measurement>& measurements,
                                           const Camera& camera) {
    Triangulator triangulator(camera);
    return triangulator.triangulateLandmark(odom_poses, measurements);
}

std::vector<Landmark> triangulate(const Dataset& dataset) {
    Triangulator triangulator(dataset.camera);
    return triangulator.triangulateFromDataset(dataset);
}

}  // namespace pms
