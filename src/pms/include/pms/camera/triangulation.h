#pragma once

#include <utility>
#include <vector>

#include "pms/camera/camera.h"
#include "pms/dataset/dataset.h"
#include "pms/dataset/landmark.h"
#include "pms/dataset/measurement.h"
#include "pms/math/definitions.h"

namespace pms {

/**
 * @brief Configuration parameters for triangulation
 */
struct TriangulationConfig {
    Scalar homogeneous_threshold = 1e-8;  ///< Threshold for homogeneous coordinate validity
    bool enable_cheirality_check = true;  ///< Enable cheirality check (point in front of camera)
    size_t min_observations = 2;          ///< Minimum number of observations required for triangulation
    Scalar max_reprojection_error = 20.0;  ///< Maximum allowed reprojection error for outlier rejection
    TriangulationConfig() = default;
};

/**
 * @brief Triangulator class for 3D landmark triangulation using Direct Linear Transform (DLT)
 *
 * This class encapsulates the triangulation functionality, implementing the DLT algorithm
 * to estimate 3D landmark positions from multiple 2D observations across different camera poses.
 * It provides both single landmark and batch triangulation methods.
 */
class Triangulator {
   private:
    Camera camera_;               ///< Camera parameters
    TriangulationConfig config_;  ///< Triangulation configuration

   public:
    /**
     * @brief Constructor with camera parameters
     * @param camera Camera parameters including intrinsics and extrinsics
     * @param config Triangulation configuration (optional)
     */
    explicit Triangulator(const Camera& camera, const TriangulationConfig& config = TriangulationConfig());

    /**
     * @brief Triangulate the 3D position of a single landmark
     *
     * Uses the Direct Linear Transform (DLT) algorithm to estimate the 3D position
     * of a landmark given its 2D measurements from multiple camera poses.
     *
     * @param odom_poses Vector of robot poses for each measurement
     * @param measurements Vector of 2D measurements of the landmark
     * @return Pair containing the triangulated 3D position and success flag
     */
    std::pair<Vector3, bool> triangulateLandmark(const std::vector<Pose3>& odom_poses,
                                                 const std::vector<Measurement>& measurements) const;

    /**
     * @brief Triangulate all landmarks from measurements grouped by landmark ID
     *
     * @param measurements_per_landmark Vector of measurement vectors, indexed by landmark ID
     * @param odom_poses All robot poses from the trajectory
     * @return Vector of triangulated landmarks with their positions and validity flags
     */
    std::vector<Landmark>
    triangulateAll(const std::vector<std::vector<Measurement>>& measurements_per_landmark,
                   const std::vector<Pose3>& odom_poses) const;

    /**
     * @brief Triangulate all landmarks in a dataset
     *
     * Convenience method that extracts measurements per landmark from the dataset
     * and triangulates all landmarks.
     *
     * @param dataset The dataset containing measurements and trajectory
     * @return Vector of triangulated landmarks with their positions and validity flags
     */
    std::vector<Landmark> triangulateFromDataset(const Dataset& dataset) const;

    /**
     * @brief Get the camera parameters
     * @return Reference to the camera parameters
     */
    const Camera& getCamera() const {
        return camera_;
    }

    /**
     * @brief Get the triangulation configuration
     * @return Reference to the configuration
     */
    const TriangulationConfig& getConfig() const {
        return config_;
    }

    /**
     * @brief Update the triangulation configuration
     * @param config New configuration parameters
     */
    void setConfig(const TriangulationConfig& config) {
        config_ = config;
    }

    /**
     * @brief Triangulate landmarks using consecutive frame pairs approach
     *
     * This method triangulates landmarks from consecutive frame pairs and averages
     * the results, providing better initial estimates through outlier filtering
     * and robust estimation. It processes frame pairs sequentially and applies
     * strict outlier rejection before averaging multiple triangulation estimates.
     * This approach is more robust than multi-view triangulation as it uses
     * well-conditioned two-view geometries and filters outliers per estimate.
     *
     * @param dataset The dataset containing trajectory and measurements
     * @return Vector of triangulated landmarks using the consecutive frame approach
     */
    std::vector<Landmark> triangulateRansac(const Dataset& dataset) const;

private:
    /**
     * @brief Triangulate a landmark from two camera views
     * @param p1 Image point in first view
     * @param p2 Image point in second view
     * @param T1 Camera pose for first view
     * @param T2 Camera pose for second view
     * @return Triangulated 3D point
     */
    Vector3 triangulateFromTwoViews(const Vector2& p1, const Vector2& p2,
                                   const Pose3& T1, const Pose3& T2) const;

    /**
     * @brief Check if a triangulated point is an outlier
     * @param triangulated_point The 3D point to check
     * @param point_2d The corresponding 2D measurement
     * @param camera_T_world Camera pose transformation
     * @return True if the point is an outlier
     */
    bool isOutlier(const Vector3& triangulated_point, const Vector2& point_2d,
                   const Pose3& camera_T_world) const;

    /**
     * @brief Compute reprojection error for a 3D point
     * @param point_3d The 3D point
     * @param point_2d The corresponding 2D measurement
     * @param camera_T_world Camera pose transformation
     * @return Reprojection error magnitude
     */
    Scalar computeReprojectionError(const Vector3& point_3d, const Vector2& point_2d,
                                   const Pose3& camera_T_world) const;
};

/**
 * @deprecated Use Triangulator class instead
 * @brief Triangulate the 3D position of a landmark given its 2D measurements
 */
std::pair<Vector3, bool> triangulateLandmark(const std::vector<Pose3>& odom_poses,
                                             const std::vector<Measurement>& measurements,
                                             const Camera& camera);

/**
 * @deprecated Use Triangulator class instead
 * @brief Triangulate all landmarks in the dataset using the Direct Linear Transform (DLT) algorithm
 */
std::vector<Landmark> triangulate(const Dataset& dataset);

/**
 * @brief Triangulate landmarks using the same approach as MATLAB initial_guess.m
 *
 * This function closely follows the MATLAB implementation, processing measurements
 * per landmark and applying the same DLT algorithm with cheirality checks.
 *
 * @param dataset The dataset containing trajectory and measurements
 * @return Vector of triangulated landmarks matching MATLAB output
 */
std::vector<Landmark> triangulateMatlab(const Dataset& dataset);

/**
 * @brief Triangulate landmarks using consecutive frame pairs approach
 *
 * This function triangulates landmarks from consecutive frame pairs and averages
 * the results, providing better initial estimates through outlier filtering
 * and robust estimation. This approach is more robust than multi-view triangulation
 * as it uses well-conditioned two-view geometries and filters outliers per estimate.
 *
 * @param dataset The dataset containing trajectory and measurements
 * @return Vector of triangulated landmarks using the consecutive frame approach
 */
std::vector<Landmark> triangulateRansac(const Dataset& dataset);

}  // namespace pms
