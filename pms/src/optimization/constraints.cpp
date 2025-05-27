#include "pms/optimization/constraints.h"

#include <iostream>
namespace pms {

PoseLandmarkConstraint::PoseLandmarkConstraint(const Pose2& Xr, const Vector3& Xl, const Vector2& z) {
    Jr = Eigen::Matrix<Scalar, 2, 3>::Zero();
    Jl = Eigen::Matrix<Scalar, 2, 3>::Zero();
    error = Vector2::Zero();
}

PosePoseConstraint::PosePoseConstraint(const Pose2& Xi, const Pose2& Xj) {
    Ji = Eigen::Matrix<Scalar, 1, 3>::Zero();
    Jj = Eigen::Matrix<Scalar, 1, 3>::Zero();
    error = Eigen::Matrix<Scalar, 1, 1>::Zero();
}

}  // namespace pms