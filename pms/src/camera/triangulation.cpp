#include "pms/camera/triangulation.h"

#include <cassert>
#include <cmath>
#include <map>
#include <set>

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
        
        A.row(i * 2) = projection.row(0) - u * projection.row(2);
        A.row(i * 2 + 1) = v * projection.row(2) - projection.row(1);
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

std::vector<Landmark> Triangulator::triangulateRansac(const Dataset& dataset) const {
    const auto& trajectory = dataset.trajectory;
    
    // Group measurements by landmark ID
    std::vector<std::vector<Measurement>> measurements_per_landmark = dataset.getMeasurementsPerLandmark();
    
    // For each landmark, collect triangulated points from consecutive frame pairs
    std::map<int, std::vector<Vector3>> triangulated_points_per_landmark;
    
    // Process consecutive frame pairs for robust estimation
    for (size_t frame_id = 0; frame_id < trajectory.size() - 1; ++frame_id) {
        const auto& current_frame = trajectory[frame_id];
        const auto& next_frame = trajectory[frame_id + 1];
        
        // Get camera poses for both frames
        Pose3 T1 = camera_.computeWorldToCamera(Pose3(current_frame.odometry.getPose2()));
        Pose3 T2 = camera_.computeWorldToCamera(Pose3(next_frame.odometry.getPose2()));
        
        // Find common landmarks between consecutive frames
        std::set<int> current_landmarks, next_landmarks;
        for (const auto& meas : current_frame.measurements) {
            current_landmarks.insert(meas.landmark_id);
        }
        for (const auto& meas : next_frame.measurements) {
            next_landmarks.insert(meas.landmark_id);
        }
        
        // Process landmarks visible in both frames
        for (int landmark_id : current_landmarks) {
            if (next_landmarks.find(landmark_id) == next_landmarks.end()) continue;
            
            // Find measurements for this landmark in both frames
            Vector2 p1, p2;
            bool found_current = false, found_next = false;
            
            for (const auto& meas : current_frame.measurements) {
                if (meas.landmark_id == landmark_id) {
                    p1 = meas.image_point;
                    found_current = true;
                    break;
                }
            }
            
            for (const auto& meas : next_frame.measurements) {
                if (meas.landmark_id == landmark_id) {
                    p2 = meas.image_point;
                    found_next = true;
                    break;
                }
            }
            
            if (!found_current || !found_next) continue;
            
            // Triangulate using two-view DLT
            Vector3 triangulated_point = triangulateFromTwoViews(p1, p2, T1, T2);
            
            // Outlier check with strict filtering
            if (!isOutlier(triangulated_point, p1, T1) && 
                !isOutlier(triangulated_point, p2, T2)) {
                triangulated_points_per_landmark[landmark_id].push_back(triangulated_point);
            }
        }
    }
    
    // Average all triangulated points for each landmark using robust estimation
    std::vector<Landmark> result_landmarks;
    result_landmarks.reserve(measurements_per_landmark.size());
    
    for (size_t i = 0; i < measurements_per_landmark.size(); ++i) {
        if (triangulated_points_per_landmark.find(i) != triangulated_points_per_landmark.end()) {
            const auto& points = triangulated_points_per_landmark[i];
            
            // Average the triangulated points
            Vector3 averaged_point = Vector3::Zero();
            for (const auto& point : points) {
                averaged_point += point;
            }
            averaged_point /= static_cast<Scalar>(points.size());
            
            result_landmarks.push_back(Landmark(averaged_point, i, true));
        } else {
            result_landmarks.push_back(Landmark(Vector3::Zero(), i, false));
        }
    }
    
    return result_landmarks;
}

Vector3 Triangulator::triangulateFromTwoViews(const Vector2& p1, const Vector2& p2, 
                                             const Pose3& T1, const Pose3& T2) const {
    // Compute projection matrices
    Eigen::Matrix<Scalar, 3, 4> m1 = camera_.computeProjectionMatrix(T1);
    Eigen::Matrix<Scalar, 3, 4> m2 = camera_.computeProjectionMatrix(T2);
    
    // Build DLT matrix using proper equation ordering
    Eigen::Matrix<Scalar, 4, 4> A;
    A.row(0) = p1.y() * m1.row(2) - m1.row(1);
    A.row(1) = m1.row(0) - p1.x() * m1.row(2);
    A.row(2) = p2.y() * m2.row(2) - m2.row(1);
    A.row(3) = m2.row(0) - p2.x() * m2.row(2);
    
    // Solve using SVD
    Eigen::JacobiSVD<Eigen::Matrix<Scalar, 4, 4>> svd(A, Eigen::ComputeFullV);
    Vector4 X_homogeneous = svd.matrixV().col(3);
    
    // Dehomogenize with numerical safety check
    if (std::abs(X_homogeneous(3)) < config_.homogeneous_threshold) {
        return Vector3::Zero();
    }
    
    Vector3 triangulated_point = X_homogeneous.head<3>() / X_homogeneous(3);
    return triangulated_point;
}

bool Triangulator::isOutlier(const Vector3& triangulated_point, const Vector2& point_2d, 
                             const Pose3& camera_T_world) const {
    // Check if point is behind camera or beyond far plane
    Vector4 point_homogeneous;
    point_homogeneous << triangulated_point, 1.0;
    Vector3 point_in_camera = (camera_T_world.getHomogen() * point_homogeneous).head<3>();
    
    if (point_in_camera.z() <= 0 || point_in_camera.z() > camera_.z_far) {
        return true;
    }
    
    // Check reprojection error with threshold matching the robust approach
    Scalar reprojection_error = computeReprojectionError(triangulated_point, point_2d, camera_T_world);
    if (reprojection_error > 10.0) {
        return true;
    }
    
    return false;
}

Scalar Triangulator::computeReprojectionError(const Vector3& point_3d, const Vector2& point_2d, 
                                             const Pose3& camera_T_world) const {
    Eigen::Matrix<Scalar, 3, 4> P = camera_.computeProjectionMatrix(camera_T_world);
    Vector4 point_homogeneous;
    point_homogeneous << point_3d, 1.0;
    
    Vector3 projected_point = P * point_homogeneous;
    projected_point /= projected_point(2);
    
    Vector2 projected_2d = projected_point.head<2>();
    return (projected_2d - point_2d).norm();
}

std::vector<Landmark> triangulate(const Dataset& dataset) {
    Triangulator triangulator(dataset.camera);
    return triangulator.triangulateFromDataset(dataset);
}

std::vector<Landmark> triangulateRansac(const Dataset& dataset) {
    Triangulator triangulator(dataset.camera);
    return triangulator.triangulateRansac(dataset);
}

}  // namespace pms
