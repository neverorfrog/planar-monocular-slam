#pragma once

#include "pms/math/definitions.h"
#include "pms/math/pose2.h"

namespace pms {
struct PoseLandmarkConstraint {
    Vector2 error;  // Error vector between the observed landmark position and the predicted position
    Eigen::Matrix<Scalar, 2, 3> Jr;  // Derivative of error with respect to robot increment. As tall as the
                                     // error function, as wide as the robot state dimension
    Eigen::Matrix<Scalar, 2, 3> Jl;  // Derivative of error with respect to landmark increment. As tall as the
                                     // error function, as wide as the landmark state dimension

    /**
     * @brief Constructor for PoseLandmarkConstraint
     * @param Xr Pose of the robot
     * @param Xl Position of the landmark
     * @param z  2D observation of the landmark in the image
     */
    PoseLandmarkConstraint(const Pose2& Xr, const Vector3& Xl, const Vector2& z);
};

struct PosePoseConstraint {
    Scalar error;                    // Euclidean distance between the two poses
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
