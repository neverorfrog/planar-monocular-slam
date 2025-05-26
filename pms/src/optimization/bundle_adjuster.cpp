#include "pms/optimization/bundle_adjuster.h"

#include <iostream>


namespace pms {

BundleAdjuster::BundleAdjuster(const Solution& solution, const Dataset& dataset,
                               const BundleAdjustmentConfig& config)
    : config(config), stats{} {
    state = State(dataset.getOdometryPoses2(), solution.world);
    camera = dataset.camera;
    // TODO: Extract flat measurements from dataset
    std::cout << "BundleAdjuster initialized with max iterations: " << config.max_iterations
              << " and tolerance: " << config.tolerance << std::endl;
    std::cout << "NUMBER OF ODOM POSES " << state.getNumPoses() << std::endl;
    std::cout << "NUMBER OF LANDMARKS " << state.getNumLandmarks() << std::endl;
}

bool BundleAdjuster::performIteration() {
    // TODO: Main optimization loop iteration
    // 1. Compute current cost
    // 2. Build normal equations
    // 3. Solve for parameter update
    // 4. Apply update using manifold operations and step size
    // 5. Check convergence criteria
    return false;
}

const BundleAdjuster::OptimizationStats& BundleAdjuster::getStats() const {
    return stats;
}

}  // namespace pms
