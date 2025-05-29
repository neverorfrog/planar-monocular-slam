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
    Scalar homogeneous_threshold = 1e-10;  ///< Threshold for homogeneous coordinate validity
    bool enable_cheirality_check = true;   ///< Enable cheirality check (point in front of camera)
    size_t min_observations = 2;           ///< Minimum number of observations required for triangulation

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
};

// Legacy functions for backward compatibility
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

}  // namespace pms
