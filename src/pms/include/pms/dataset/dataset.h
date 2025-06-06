#pragma once

#include <string>
#include <vector>

#include "pms/camera/camera.h"
#include "pms/dataset/landmark.h"
#include "pms/dataset/measurement.h"
#include "pms/math/definitions.h"
#include "pms/dataset/trajectory_point.h"

namespace pms {

/**
 * @brief Complete dataset for SLAM containing camera, landmarks, and trajectory
 * 
 * The Dataset class encapsulates all data needed for SLAM algorithms including:
 * - Camera parameters and calibration
 * - World landmarks (3D features)
 * - Trajectory with odometry, ground truth, and measurements
 * 
 * This class handles loading all data from a folder structure and provides
 * convenient access methods for different data components.
 */
class Dataset {
private:
    /**
     * @brief Load camera calibration data from file
     * @param folderpath Path to dataset folder
     * @return Camera object with loaded parameters
     * @throws std::runtime_error if file cannot be read or parsed
     */
    Camera loadCameraData(const std::string& folderpath);

    /**
     * @brief Load world landmarks from file
     * @param folderpath Path to dataset folder
     * @return Vector of Landmark objects
     * @throws std::runtime_error if file cannot be read or parsed
     */
    std::vector<Landmark> loadWorldData(const std::string& folderpath);

    /**
     * @brief Load trajectory with measurements from files
     * @param folderpath Path to dataset folder
     * @return Vector of TrajPoint objects with associated measurements
     * @throws std::runtime_error if files cannot be read or parsed
     */
    std::vector<TrajPoint> loadTrajectory(const std::string& folderpath);

public:
    Camera camera;                      ///< Camera parameters and pose
    std::vector<Landmark> world;        ///< World landmarks
    std::vector<TrajPoint> trajectory;  ///< Trajectory points with measurements

    /**
     * @brief Construct dataset by loading from folder
     * @param folderpath Path to folder containing dataset files
     * @throws std::runtime_error if folder doesn't exist or files cannot be loaded
     * 
     * Expected folder structure:
     * - camera.dat: Camera calibration parameters
     * - world.dat: Landmark positions
     * - trajectory.dat: Trajectory poses
     * - meas-*.dat: Measurement files
     */
    explicit Dataset(const std::string& folderpath);

    /**
     * @brief Extract odometry poses from trajectory
     * @return Vector of odometry poses
     */
    std::vector<Pose3> getOdometryPoses3() const;

    /**
     * @brief Extract ground truth poses from trajectory
     * @return Vector of ground truth poses
     */
    std::vector<Pose3> getGroundTruthPoses3() const;

    /**
     * @brief Extract odometry poses in 2D (for planar SLAM)
     * @return Vector of Pose2 objects representing odometry poses
     */
    std::vector<Pose2> getOdometryPoses2() const;

    /**
     * @brief Extract ground truth poses in 2D (for planar SLAM)
     * @return Vector of Pose2 objects representing ground truth poses
     */
    std::vector<Pose2> getGroundTruthPoses2() const;

    /**
     * @brief Extract landmark positions from world data
     * @return Vector of 3D landmark positions
     */
    std::vector<Vector3> getLandmarkPositions() const;

    /**
     * @brief Get all landmark positions as a vector of Vector3
     * @return Vector of 3D landmark positions
     */
    std::vector<std::vector<Measurement>> getMeasurementsPerLandmark() const;


    /**
     * @brief Get all measurements in a flat vector
     * @param landmarks Vector of landmarks to filter measurements 
     * @return Vector of Measurement objects containing all measurements
     */
    std::vector<Measurement> getFlatMeasurements(const std::vector<Landmark>& landmarks) const;

    /**
     * @brief Convert dataset to string representation
     * @return String containing formatted dataset information
     */
    std::string toString() const;
};

}  // namespace pms
