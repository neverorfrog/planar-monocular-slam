#pragma once

#include <sstream>
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
    Pose3 pose;             ///< Camera pose in world coordinates
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
          z_near(other.z_near),
          z_far(other.z_far),
          width(other.width),
          height(other.height) {}

    /**
     * @brief Assignment operator
     * @param other Camera object to assign from
     * @return Reference to this camera
     */
    const Camera& operator=(const Camera& other) {
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

    /**
     * @brief Convert camera parameters to string representation
     * @return String containing formatted camera information
     */
    std::string toString() const {
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
};

}  // namespace pms
