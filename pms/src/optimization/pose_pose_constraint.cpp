#include "pms/optimization/pose_pose_constraint.h"

namespace pms {

PosePoseConstraint::PosePoseConstraint(const Pose2& Xi, const Pose2& Xj) {
    Ji = Eigen::Matrix<Scalar, 1, 3>::Zero();
    Jj = Eigen::Matrix<Scalar, 1, 3>::Zero();
    error = Eigen::Matrix<Scalar, 1, 1>::Zero();
}

}  // namespace pms