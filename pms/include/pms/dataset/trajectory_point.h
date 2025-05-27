#pragma once

#include <sstream>
#include <string>
#include <vector>

#include "pms/math/pose3.h"
#include "pms/dataset/measurement.h"

namespace pms {

/**
 * @brief Represents a single point in a trajectory with poses and measurements
 * 
 * A trajectory point contains both odometry (estimated) and ground truth poses,
 * along with all landmark measurements taken at this position. This structure
 * is fundamental for SLAM algorithms that compare estimated trajectories with
 * ground truth data.
 */
struct TrajPoint {
    int id;                                ///< Unique identifier for this trajectory point
    Pose3 odometry;                        ///< Estimated pose from odometry
    Pose3 ground_truth;                    ///< Ground truth pose (if available)
    std::vector<Measurement> measurements; ///< All landmark measurements at this pose

    /**
     * @brief Default constructor
     * 
     * Initializes ID to zero and poses to identity
     */
    TrajPoint() : id(0) {}

    /**
     * @brief Constructor with poses
     * @param id Unique identifier for this trajectory point
     * @param odom Estimated pose from odometry
     * @param gt Ground truth pose
     */
    TrajPoint(int id, const Pose3& odom, const Pose3& gt)
        : id(id), odometry(odom), ground_truth(gt) {}

    /**
     * @brief Copy constructor
     * @param other TrajPoint object to copy from
     */
    TrajPoint(const TrajPoint& other)
        : id(other.id),
          odometry(other.odometry),
          ground_truth(other.ground_truth),
          measurements(other.measurements) {}

    /**
     * @brief Assignment operator
     * @param other TrajPoint object to assign from
     * @return Reference to this trajectory point
     */
    const TrajPoint& operator=(const TrajPoint& other) {
        if (this != &other) {
            id = other.id;
            odometry = other.odometry;
            ground_truth = other.ground_truth;
            measurements = other.measurements;
        }
        return *this;
    }

    /**
     * @brief Convert trajectory point to string representation
     * @return String containing formatted trajectory point information
     */
    std::string toString() const {
        std::ostringstream oss;
        oss << "Trajectory Point ID: " << id << "\n"
            << "Odometry: " << odometry.toString() << "\n"
            << "Ground Truth: " << ground_truth.toString() << "\n"
            << "Measurements:\n";
        for (const auto& meas : measurements) {
            oss << meas.toString() << "\n";
        }
        return oss.str();
    }
};

}  // namespace pms
