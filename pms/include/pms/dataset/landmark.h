#pragma once

#include <sstream>
#include <stdexcept>
#include <string>

#include "pms/math/definitions.h"

namespace pms {

/**
 * @brief Represents a 3D landmark in the world coordinate system
 * 
 * A landmark is a distinctive feature in the environment that can be observed
 * from multiple camera poses. Each landmark has a unique identifier, 3D position,
 * and validity flag.
 */
struct Landmark {
    Vector3 position;  ///< 3D position in world coordinates
    int id;            ///< Unique identifier for the landmark
    bool valid;        ///< Flag indicating if the landmark is valid/observable

    /**
     * @brief Constructor with landmark ID only
     * @param landmark_id Unique identifier for the landmark
     * 
     * Creates a landmark at origin with invalid status
     */
    Landmark(int landmark_id) : position(Vector3::Zero()), id(landmark_id), valid(false) {}

    /**
     * @brief Constructor with full parameters
     * @param pos 3D position of the landmark
     * @param landmark_id Unique identifier for the landmark
     * @param valid Validity flag for the landmark
     * @throws std::invalid_argument if position is not 3D
     */
    Landmark(const Vector3& pos, int landmark_id, bool valid) : position(pos), id(landmark_id), valid(valid) {
        if (pos.size() != 3) {
            throw std::invalid_argument("Landmark position must be a 3D vector.");
        }
    }

    /**
     * @brief Copy constructor
     * @param other Landmark object to copy from
     */
    Landmark(const Landmark& other) : position(other.position), id(other.id), valid(other.valid) {}

    /**
     * @brief Assignment operator
     * @param other Landmark object to assign from
     * @return Reference to this landmark
     */
    const Landmark& operator=(const Landmark& other) {
        if (this != &other) {
            position = other.position;
            id = other.id;
            valid = other.valid;
        }
        return *this;
    }

    /**
     * @brief Equality comparison operator
     * @param other Landmark to compare with
     * @return True if landmarks have same ID and approximately same position
     */
    bool operator==(const Landmark& other) const {
        return id == other.id && position.isApprox(other.position);
    }

    /**
     * @brief Convert landmark to string representation
     * @return String containing formatted landmark information
     */
    std::string toString() const {
        std::ostringstream oss;
        oss << "Landmark ID: " << id << "\n"
            << "Position: " << position.transpose();
        return oss.str();
    }
};

}  // namespace pms
