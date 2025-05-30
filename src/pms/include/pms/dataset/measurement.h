#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

#include "pms/math/definitions.h"

namespace pms {

/**
 * @brief Represents a 2D measurement of a landmark in an image
 *
 * A measurement connects a 3D landmark to its 2D projection in a camera image.
 * It contains information about which trajectory point (sequence number) the
 * measurement was taken from, which landmark was observed, and the pixel coordinates.
 */
struct Measurement {
    int pose_id;          ///< ID of the pose (inside the trajectory)
    int landmark_id;      ///< ID of the observed landmark
    Vector2 image_point;  ///< 2D pixel coordinates in the image

    /**
     * @brief Default constructor
     *
     * Initializes all IDs to zero and image point to origin
     */
    Measurement() : pose_id(0), landmark_id(0) {}

    /**
     * @brief Constructor with full parameters
     * @param pose_id Sequence number of the trajectory point
     * @param lm_id ID of the observed landmark
     * @param img_pt 2D pixel coordinates in the image
     * @throws std::invalid_argument if image_point is not 2D
     */
    Measurement(int pose_id, int lm_id, const Vector2& img_pt)
        : pose_id(pose_id), landmark_id(lm_id), image_point(img_pt) {
        if (img_pt.size() != 2) {
            throw std::invalid_argument("Measurement image_point must be a 2D vector.");
        }
    }

    /**
     * @brief Copy constructor
     * @param other Measurement object to copy from
     */
    Measurement(const Measurement& other)
        : pose_id(other.pose_id), landmark_id(other.landmark_id), image_point(other.image_point) {}

    /**
     * @brief Assignment operator
     * @param other Measurement object to assign from
     * @return Reference to this measurement
     */
    const Measurement& operator=(const Measurement& other) {
        if (this != &other) {
            pose_id = other.pose_id;
            landmark_id = other.landmark_id;
            image_point = other.image_point;
        }
        return *this;
    }

    /**
     * @brief Equality comparison operator
     * @param other Measurement to compare with
     * @return True if all fields match (with approximate comparison for image_point)
     */
    bool operator==(const Measurement& other) const {
        return pose_id == other.pose_id && landmark_id == other.landmark_id
               && image_point.isApprox(other.image_point);
    }

    /**
     * @brief Check if this measurement is near another in image space
     * @param other Measurement to compare with
     * @param threshold Distance threshold in pixels
     * @return True if measurements are within threshold distance
     */
    bool isNear(const Measurement& other, Scalar threshold) const {
        return (image_point - other.image_point).norm() < threshold;
    }

    /**
     * @brief Convert measurement to string representation
     * @return String containing formatted measurement information
     */
    std::string toString() const {
        std::ostringstream oss;
        oss << "Measurement:\n"
            << "Pose ID: " << pose_id << "\n"
            << "Landmark ID: " << landmark_id << "\n"
            << "Image Point: " << image_point.transpose();
        return oss.str();
    }
};

}  // namespace pms
