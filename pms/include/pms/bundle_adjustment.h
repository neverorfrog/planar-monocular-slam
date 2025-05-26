#pragma once

#include "pms/dataset.h"
#include "pms/optimization/state.h"
#include "pms/types/solution.h"
#include "pms/utils.h"
#include <iostream>

namespace pms {

/**
 * @brief High-level bundle adjustment interface
 *
 * Provides a convenient interface to perform bundle adjustment on
 * the existing Solution and Dataset types. Handles conversion between
 * the optimization state and the standard data structures.
 *
 * Bundle adjustment optimizes camera poses and landmark positions by
 * minimizing reprojection errors through pose-landmark constraints
 * and pose-pose constraints from odometry.
 */

/**
 * @brief Perform bundle adjustment on a SLAM solution
 *
 * Takes an initial solution (from triangulation) and refines
 * both camera poses and landmark positions using bundle adjustment.
 * For planar SLAM, only the x,y,theta components of poses are optimized.
 *
 * Uses pose-landmark constraints from image measurements and
 * pose-pose constraints from odometry to improve the solution.
 *
 * @param solution [in/out] SLAM solution to optimize
 * @param dataset Dataset containing measurements and camera parameters
 * @return True if optimization was successful
 */
inline bool bundleAdjust(Solution& solution, const Dataset& dataset) {
    std::cout << "LETSSA GO" << std::endl;
    // 1. Extract trajectory from the dataset and initial guess from
    //    solution.
    std::vector<Pose2> odom_poses = dataset.getOdometryPoses2();
    std::vector<Landmark> landmarks = solution.world;
    State state{odom_poses, landmarks};

    for(const auto& landmark : landmarks){
        std::cout << landmark.position.format(PythonFmt) << std::endl;
    }

    std::cout << "NUMBER OF ODOM POSES " << state.getNumPoses() << std::endl;
    std::cout << "NUMBER OF LANDMARKS " << state.getNumLandmarks() << std::endl;

    // TODO: Continue with ba implementation (maybe use class BundleAdjuster?)

    return false;
}

}  // namespace pms
