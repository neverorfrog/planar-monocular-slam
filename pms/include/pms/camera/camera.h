#pragma once

#include <string>

#include "pms/math/definitions.h"
#include "pms/math/pose3.h"

namespace pms {

/**
 * @brief Represents a camera with intrinsic and extrinsic parameters
 *
 * This struct encapsulates all the necessary parameters for a camera model including
 * the camera matrix (intrinsic parameters), pose (extrinsic parameters), and
 * viewport dimensions.
 */
struct Camera {
    Matrix3 camera_matrix;  ///< Camera intrinsic matrix (3x3)
    Pose3 pose;             ///< Camera pose in robot frame
    Pose3 inverse_pose;     ///< Robot pose in camera frame (inverse of pose)
    Scalar z_near;          ///< Near clipping plane distance
    Scalar z_far;           ///< Far clipping plane distance
    int width;              ///< Image width in pixels
    int height;             ///< Image height in pixels

    /**
     * @brief Default constructor
     *
     * Initializes camera with zero values for scalar parameters
     */
    Camera() : z_near(0), z_far(0), width(0), height(0) {}

    /**
     * @brief Copy constructor
     * @param other Camera object to copy from
     */
    Camera(const Camera& other)
        : camera_matrix(other.camera_matrix),
          pose(other.pose),
          inverse_pose(other.inverse_pose),
          z_near(other.z_near),
          z_far(other.z_far),
          width(other.width),
          height(other.height) {}

    /**
     * @brief Assignment operator
     * @param other Camera object to assign from
     * @return Reference to this camera
     */
    const Camera& operator=(const Camera& other);

    /**
     * @brief Compute the camera's world-to-camera transformation
     * @param robot_pose Pose of the robot in world coordinates
     * @return Pose3 representing the transformation from world to camera coordinates
     */
    const Pose3 computeWorldToCamera(const Pose3& robot_pose) const;

    /**
     * @brief Compute the projection matrix for the camera
     * @param camera_T_world Pose of the world in camera coordinates
     * @return 3x4 projection matrix combining intrinsic and extrinsic parameters
     */
    const Eigen::Matrix<Scalar, 3, 4> computeProjectionMatrix(const Pose3& camera_T_world) const;

    /**
     * @brief Project a 3D point into the camera's frame
     * @param point 3D point in world coordinates
     * @return 3D point in camera coordinates
     */
    const Vector3 pointInCamera(const Vector3& point,
                                const Eigen::Matrix<Scalar, 3, 4>& projection_matrix) const;

    /**
     * @brief Project a 3D point onto the image plane
     * @param point 3D point in world coordinates
     * @return 2D point in image coordinates and a validity flag
     */
    const std::pair<Vector2, bool> projectToImage(const Vector3& point) const;

    /**
     * @brief Convert camera parameters to string representation
     * @return String containing formatted camera information
     */
    std::string toString() const;
};

}  // namespace pms
