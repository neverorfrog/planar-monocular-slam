#pragma once

#include "pms/math/definitions.h"
#include "pms/math/pose2.h"

namespace pms {

struct PosePoseConstraint {
    Eigen::Matrix<Scalar, 1, 1> error;  // Euclidean distance between the two poses
    Eigen::Matrix<Scalar, 1, 3> Ji;  // Derivative of error with respect to the first pose increment. As tall
                                     // as the error function, as wide as the robot state dimension
    Eigen::Matrix<Scalar, 1, 3> Jj;  // Derivative of error with respect to the second pose increment. As tall
                                     // as the error function, as wide as the robot state dimension
    Scalar chi = 0.0;                // Chi-squared value for this constraint, used for robust estimation
    Scalar max_chi = 1e3;   // Maximum chi-squared value for this constraint, used for robust estimation
    bool is_inlier = true;  // Whether this constraint is considered an inlier based on the chi-squared value
    Scalar h;               // Predicted measurement (euclidean distance between the two poses)

    /**
     * @brief Constructor for PosePoseConstraint
     * @param Xi First pose in the constraint
     * @param Xj Second pose in the constraint
     * @param z Movement between two odometry poses
     */
    PosePoseConstraint(const Pose2& Xi, const Pose2& Xj, const Pose2& z);
};
}  // namespace pms
