#pragma once

#include <vector>

#include "pms/camera/camera.h"
#include "pms/dataset/dataset.h"
#include "pms/math/definitions.h"
#include "pms/optimization/state.h"

namespace pms {

/**
 * @brief Configuration parameters for bundle adjustment
 */
struct BundleAdjustmentConfig {
    int max_iterations = 100;   ///< Maximum number of iterations
    Scalar tolerance = 1e-6;    ///< Convergence tolerance
    bool pose_landmark = true;  ///< Whether to use pose-landmark constraints
    bool pose_pose = true;      ///< Whether to use pose-pose constraints
};

/**
 * @brief Bundle adjustment optimizer using Gauss-Newton algorithm
 *
 * Implements non-linear least squares optimization for simultaneous pose
 * and landmark estimation in SLAM. Uses sparse matrix operations for
 * efficiency with large problems.
 *
 * Supports two types of constraints:
 * - Pose-landmark constraints: Connect SE(2) robot poses to 3D landmarks
 *   through camera projection and image measurements
 * - Pose-pose constraints: Connect consecutive poses through odometry
 *   measurements
 */
class BundleAdjuster {
   public:
    /**
     * @brief Constructor with configuration
     * @param dataset Dataset containing camera, landmarks, and trajectory
     * @param config Bundle adjustment configuration parameters
     */
    explicit BundleAdjuster(const std::vector<Landmark>& landmarks, const Dataset& dataset,
                            const BundleAdjustmentConfig& config = BundleAdjustmentConfig{});

    /**
     * @brief Optimization statistics at the end of an iteration
     * @return Structure containing optimization results
     */
    struct OptimizationStats {
        int num_iterations;   ///< Number of iterations performed
        Scalar pose_chi;      ///< PosePoseConstraint
        Scalar landmark_chi;  ///< PoseLandmarkConstraint
        Scalar num_inliers;   ///< Number of inliers in the current iteration
        bool converged;       ///< Whether optimization converged

        std::string toString() const {
            return "OptimizationStats{iterations: " + std::to_string(num_iterations) + ", pose_chi: "
                   + std::to_string(pose_chi) + ", landmark_chi: " + std::to_string(landmark_chi)
                   + ", converged: " + (converged ? "true" : "false") + "}";
        }
    };

    /**
     * @brief Main optimization function
     * @return True if convergence criteria are met, false otherwise
     */
    const OptimizationStats& performIteration();

    /**
     * @brief Get the current optimization state
     * @return Current state of the optimization
     */
    const State& getState() const {
        return state;
    }

   private:
    BundleAdjustmentConfig config;     ///< Configuration parameters
    OptimizationStats stats;           ///< Optimization statistics
    State state;                       ///< Current optimization state
    Camera camera;                     ///< Camera parameters for projection
    Dataset dataset;                   ///< Dataset containing measurements and trajectory

};

}  // namespace pms
