#include "pms/camera/camera.h"

#include <sstream>

namespace pms {

const Camera& Camera::operator=(const Camera& other) {
    if (this != &other) {
        camera_matrix = other.camera_matrix;
        pose = other.pose;
        inverse_pose = other.inverse_pose;
        z_near = other.z_near;
        z_far = other.z_far;
        width = other.width;
        height = other.height;
    }
    return *this;
}

const Pose3 Camera::computeWorldToCamera(const Pose3& robot_pose) const {
    const Pose3 world_T_camera = (robot_pose * this->pose); // transforms from camera frame to world frame
    const Pose3 camera_T_world = world_T_camera.inverse();  // transforms from world frame to camera frame
    return camera_T_world;
}

const Eigen::Matrix<Scalar, 3, 4> Camera::computeProjectionMatrix(const Pose3& camera_T_world) const {
    Eigen::Matrix<Scalar, 3, 4> projection_matrix = Eigen::Matrix<Scalar, 3, 4>::Zero();
    projection_matrix.block(0, 0, 3, 3) = camera_T_world.rotation;
    projection_matrix.block(0, 3, 3, 1) = camera_T_world.translation;
    projection_matrix = camera_matrix * projection_matrix;
    return projection_matrix;
}

const Vector3 Camera::pointInCamera(const Vector3& point, const Eigen::Matrix<Scalar, 3, 4>& projection_matrix) const {
    Vector4 homo_point = Vector4(point.x(), point.y(), point.z(), 1.0);
    Vector3 point_in_camera = projection_matrix * homo_point;
    return point_in_camera;
}

const std::pair<Vector2, bool> Camera::projectToImage(const Vector3& point) const {
    Vector2 h = Vector2(point.x() / point.z(), point.y() / point.z());
    bool is_valid = true;

    // Check if behind camera or too far
    if (point.z() < z_near || point.z() > z_far)
        is_valid = false;

    // Check if in fov
    if (h.x() > width || h.x() < 0 || h.y() > height || h.y() < 0)
        is_valid = false;

    return std::make_pair(h, is_valid);
}


std::string Camera::toString() const {
    std::ostringstream oss;
    oss << "Camera:\n"
        << "Camera Matrix:\n" << camera_matrix << "\n"
        << "Pose:\n" << pose.toString() << "\n"
        << "z_near: " << z_near << "\n"
        << "z_far: " << z_far << "\n"
        << "Width: " << width << "\n"
        << "Height: " << height;
    return oss.str();
}

}  // namespace pms