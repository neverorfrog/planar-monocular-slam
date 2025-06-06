#pragma once

#include "pms/camera/camera.h"
#include "pms/math/definitions.h"
#include "pms/math/pose2.h"

namespace pms {
struct PoseLandmarkConstraint {
    Eigen::Matrix<Scalar, 2, 1> error;  // Error vector between measurement and prediction
    Eigen::Matrix<Scalar, 2, 3> Jr;     // Derivative of error with respect to robot increment. As tall as the
                                        // error function, as wide as the robot state dimension
    Eigen::Matrix<Scalar, 2, 3> Jl;  // Derivative of error with respect to landmark increment. As tall as the
                                     // error function, as wide as the landmark state dimension
    Scalar chi = 0.0;                // Chi-squared value for this constraint, used for robust estimation
    Scalar max_chi = 5e3;   // Maximum chi-squared value for this constraint, used for robust estimation
    bool is_inlier = true;  // Whether this constraint is considered an inlier based on the chi-squared value
    Vector2 h;              // Predicted measurement in the image plane
    Eigen::Matrix<Scalar, 2, 2> omega;

    /**
     * @brief Constructor for PoseLandmarkConstraint
     * @param Xr Pose of the robot
     * @param Xl Position of the landmark
     * @param z  2D observation of the landmark in the image
     * @param camera Camera parameters for projection
     */
    PoseLandmarkConstraint(const Pose2& Xr, const Vector3& Xl, const Vector2& z, const Camera& camera);

   private:
    /**
     * @brief Computes the derivative of a rotation matrix around the z axis, given the theta angle
     * @param theta Angle of rotation around the z axis
     * @return Derivative of the rotation matrix with respect to theta
     */
    Eigen::Matrix<Scalar, 3, 3> computeRotationDerivativeZ(Scalar theta) const;
};

}  // namespace pms
