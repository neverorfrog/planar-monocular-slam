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

    /**
     * @brief Constructor for PosePoseConstraint
     * @param Xi First pose in the constraint
     * @param Xj Second pose in the constraint
     */
    PosePoseConstraint(const Pose2& Xi, const Pose2& Xj);
};
}  // namespace pms
