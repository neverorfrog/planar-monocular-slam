#pragma once

#include <vector>

#include "pms/math/definitions.h"
#include "pms/math/pose2.h"
#include "pms/types/landmark.h"

namespace pms {

/**
 * @brief Encapsulates the complete state for bundle adjustment
 *
 * Contains all robot poses (SE(2) for planar SLAM) and landmark positions (3D)
 * that need to be optimized, along with indexing information to map measurements
 * to specific poses and landmarks.
 */
class State {
   public:
    std::vector<Pose2> robot_poses;           ///< Robot poses in SE(2) for planar SLAM
    std::vector<Landmark> landmarks;          ///< 3D landmarks

    /**
     * @brief Default constructor
     */
    State() = default;

    /**
     * @brief Constructor from measurements and initial estimates
     * @param initial_poses Initial camera pose estimates
     * @param initial_landmarks Initial landmark position estimates
     */
    State(const std::vector<Pose2>& initial_poses, const std::vector<Landmark>& initial_landmarks);

    /**
     * @brief Get the total number of camera poses
     * @return Number of camera poses in the state
     */
    size_t getNumPoses() const;

    /**
     * @brief Get the total number of valid landmarks
     * @return Number of valid landmarks in the state
     */
    size_t getNumLandmarks() const;

    /**
     * @brief Get the total dimension of the optimization state
     * @return Total dimension (3 * num_poses + 3 * num_landmarks for planar SLAM)
     */
    int getStateDimension() const;

    /**
     * @brief Convert state to parameter vector for optimization
     * @return Concatenated parameter vector [poses; landmarks]
     */
    VectorX toParameterVector() const;

    /**
     * @brief Update state from parameter vector
     * @param parameters Optimization parameter vector
     */
    void fromParameterVector(const VectorX& parameters);

    /**
     * @brief Apply parameter update using manifold operations
     * @param delta_parameters Parameter space perturbation
     */
    void applyIncrement(const VectorX& delta);
};

}  // namespace pms
