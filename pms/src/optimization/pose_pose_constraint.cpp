#include "pms/optimization/pose_pose_constraint.h"

#include "pms/math/rotation_matrix.h"

namespace pms {

PosePoseConstraint::PosePoseConstraint(const Pose2& Xi, const Pose2& Xj, const Pose2& z) {
    Ji = Eigen::Matrix<Scalar, 1, 3>::Zero();
    Jj = Eigen::Matrix<Scalar, 1, 3>::Zero();
    error = Eigen::Matrix<Scalar, 1, 1>::Zero();

    h = (Xi.inverse() * Xj).translation.norm();  // Relative pose between the two poses
    Scalar meas = z.translation.norm();          // Measurement of the relative pose
    error(0) = h - meas;        // Error is the difference between the predicted and measured relative pose
    chi = error.squaredNorm();  // Chi-squared value is the squared error

    // Robust estimation
    if (chi > max_chi) {
        is_inlier = false;
        error *= std::sqrt(max_chi / chi);
        chi = max_chi;
    } else {
        is_inlier = true;
    }

    Eigen::Matrix<Scalar, 2, 2> R_i = RotationMatrix::aroundZ(Xi.rotation).matrix().block<2, 2>(0, 0);

    Jj.segment<2>(0) = Vector2(1, 1).transpose() * R_i.transpose();
    Jj(2) = Xj.translation.transpose() * R_i.transpose() * Vector2(1, -1);
    Ji = -Jj;  // Ji is the negative of Jj since we are computing the relative pose
}

}  // namespace pms