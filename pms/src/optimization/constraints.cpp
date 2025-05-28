#include "pms/optimization/constraints.h"

#include <iostream>

#include "pms/camera/camera.h"
namespace pms {

PoseLandmarkConstraint::PoseLandmarkConstraint(const Pose2& Xr, const Vector3& Xl, const Vector2& z,
                                               const Camera& camera) {
    // Compute predicted measurement
    Pose3 camera_T_world = camera.computeWorldToCamera(Pose3(Xr));
    Eigen::Matrix<Scalar, 3, 4> projection_matrix = camera.computeProjectionMatrix(camera_T_world);
    Vector3 point_in_camera = camera.pointInCamera(Xl, projection_matrix);
    h = Vector2(point_in_camera.x() / point_in_camera.z(), point_in_camera.y() / point_in_camera.z());

    // Error
    error = z - h;
    chi = error.squaredNorm();
    std::cout << "\n\nMeasurement: " << z.transpose() << std::endl;
    std::cout << "Prediction: " << h.transpose() << std::endl;
    std::cout << "Chi: " << chi << std::endl;

    // Robust estimation
    if (chi > max_chi) {
        is_inlier = false;
        return;
    }

    // Jacobian with respect to robot
    Eigen::Matrix<Scalar, 2, 3> J_proj = Eigen::Matrix<Scalar, 2, 3>::Zero();
    J_proj(0, 0) = 1.0 / point_in_camera.z();
    J_proj(1, 1) = 1.0 / point_in_camera.z();
    J_proj(0, 2) = -point_in_camera.x() / (point_in_camera.z() * point_in_camera.z());
    J_proj(1, 2) = -point_in_camera.y() / (point_in_camera.z() * point_in_camera.z());
    Eigen::Matrix<Scalar, 3, 3> J_icp_r = Eigen::Matrix<Scalar, 3, 3>::Zero();
    J_icp_r.block<3, 2>(0, 0) = -camera_T_world.rotation.block<3, 2>(0, 0);
    Vector3 robot_to_landmark = Xl - Pose3(Xr).translation;
    Matrix3 rot_derivative = camera.inverse_pose.rotation.matrix() * computeRotationDerivativeZ(Xr.rotation);
    J_icp_r.block<3, 1>(0, 2) = rot_derivative * robot_to_landmark;
    Jr = J_proj * camera.camera_matrix * J_icp_r;

    // Jacobian with respect landmark
    Jl = J_proj * camera.camera_matrix * camera_T_world.rotation;
}

Eigen::Matrix<Scalar, 3, 3> PoseLandmarkConstraint::computeRotationDerivativeZ(Scalar theta) const {
    Eigen::Matrix<Scalar, 3, 3> J_rot = Eigen::Matrix<Scalar, 3, 3>::Zero();
    J_rot(0, 0) = -sin(theta);
    J_rot(0, 1) = cos(theta);
    J_rot(1, 0) = -cos(theta);
    J_rot(1, 1) = -sin(theta);
    return J_rot;
}

PosePoseConstraint::PosePoseConstraint(const Pose2& Xi, const Pose2& Xj) {
    Ji = Eigen::Matrix<Scalar, 1, 3>::Zero();
    Jj = Eigen::Matrix<Scalar, 1, 3>::Zero();
    error = Eigen::Matrix<Scalar, 1, 1>::Zero();
}

}  // namespace pms