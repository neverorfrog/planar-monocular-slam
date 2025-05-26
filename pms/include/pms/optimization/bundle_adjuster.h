#pragma once

#include <vector>

#include "pms/dataset.h"
#include "pms/optimization/state.h"
#include "pms/types/camera.h"
#include "pms/types/solution.h"

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
     * @param config Bundle adjustment configuration parameters
     */
    explicit BundleAdjuster(const Solution& solution, const Dataset& dataset,
                            const BundleAdjustmentConfig& config = BundleAdjustmentConfig{});

    /**
     * @brief Main optimization function
     * @return True if convergence criteria are met, false otherwise
     */
    bool performIteration();

    /**
     * @brief Get the final optimization statistics
     * @return Structure containing optimization results
     */
    struct OptimizationStats {
        int num_iterations;          ///< Number of iterations performed
        Scalar initial_cost;         ///< Initial total cost
        Scalar final_cost;           ///< Final total cost
        Scalar final_gradient_norm;  ///< Final gradient norm
        bool converged;              ///< Whether optimization converged

        std::string toString() const {
            return "OptimizationStats{iterations: " + std::to_string(num_iterations) +
                   ", initial_cost: " + std::to_string(initial_cost) +
                   ", final_cost: " + std::to_string(final_cost) +
                   ", gradient_norm: " + std::to_string(final_gradient_norm) +
                   ", converged: " + (converged ? "true" : "false") + "}";
        }
    };

    const OptimizationStats& getStats() const;

   private:
    BundleAdjustmentConfig config;          ///< Configuration parameters
    OptimizationStats stats;                ///< Optimization statistics
    State state;                            ///< Current optimization state
    std::vector<Measurement> measurements;  ///< Flat measurement vector for optimization
    Camera camera;                          ///< Camera parameters for projection
};

}  // namespace pms
