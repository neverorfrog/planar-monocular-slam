#include "pms/camera/camera.h"

#include <sstream>

namespace pms {

const Camera& Camera::operator=(const Camera& other) {
    if (this != &other) {
        camera_matrix = other.camera_matrix;
        pose = other.pose;
        z_near = other.z_near;
        z_far = other.z_far;
        width = other.width;
        height = other.height;
    }
    return *this;
}

const Pose3 Camera::computeWorldToCamera(const Pose3& robot_pose) const {
    const Pose3 world_T_camera = (robot_pose * pose); // transforms from camera frame to world frame
    const Pose3 camera_T_world = world_T_camera.inverse();  // transforms from world frame to camera frame
    return camera_T_world;
}

const Eigen::Matrix<Scalar, 3, 4> Camera::computeProjectionMatrix(const Pose3& camera_T_world) const {
    Eigen::Matrix<Scalar, 3, 4> projection = Eigen::Matrix<Scalar, 3, 4>::Zero();
    projection.block(0, 0, 3, 3) = camera_T_world.rotation;
    projection.block(0, 3, 3, 1) = camera_T_world.translation;
    projection = camera_matrix * projection;
    return projection;
}

const Vector2 Camera::projectPoint(const Vector3& point, const Eigen::Matrix<Scalar, 3, 4>& projection_matrix) const {
    Vector4 homo_point = Vector4(point.x(), point.y(), point.z(), 1.0);
    Vector3 point_in_image = projection_matrix * homo_point;
    Vector2 projection = Vector2(point_in_image.x() / point_in_image.z(), point_in_image.y() / point_in_image.z());
    return projection;
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