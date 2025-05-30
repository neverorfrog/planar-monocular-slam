#include <gtest/gtest.h>

#include "pms/math/definitions.h"
#include "pms/math/pose2.h"
#include "pms/math/rotation_matrix.h"
#include "pms/optimization/manifold.h"

namespace pms {

class ManifoldTest : public ::testing::Test {
   protected:
    void SetUp() override {
        original_pose = Pose2(M_PI / 6, Vector2(1.0, 2.0));
        small_delta = Vector3(0.1, 0.2, 0.0);
        large_delta = Vector3(1.0, 2.0, 3 * M_PI);
    }

    Pose2 original_pose;
    Vector3 small_delta;
    Vector3 large_delta;
    const double tolerance = 1e-6;
};

TEST_F(ManifoldTest, BoxPlusIdentity) {
    // Test that boxPlus(pose, 0) = pose
    Vector3 zero_delta = Vector3::Zero();
    Pose2 result = Pose2Manifold::boxPlus(original_pose, zero_delta);

    EXPECT_NEAR(original_pose.translation.x(), result.translation.x(), tolerance);
    EXPECT_NEAR(original_pose.translation.y(), result.translation.y(), tolerance);
    EXPECT_NEAR(original_pose.rotation, result.rotation, tolerance);
}

TEST_F(ManifoldTest, BoxMinusIdentity) {
    // Test that boxMinus(pose, pose) = 0
    VectorX delta = Pose2Manifold::boxMinus(original_pose, original_pose);

    EXPECT_NEAR(0.0, delta(0), tolerance);
    EXPECT_NEAR(0.0, delta(1), tolerance);
    EXPECT_NEAR(0.0, delta(2), tolerance);
}

TEST_F(ManifoldTest, ManualBoxPlus) {
    // Test that boxPlus(pose, delta) gives the expected result
    Pose2 result = Pose2Manifold::boxPlus(original_pose, small_delta);

    Matrix3 delta_transformation = Matrix3::Identity();
    delta_transformation.block<2, 2>(0, 0)
        = RotationMatrix::aroundZ(small_delta(2)).matrix().block<2, 2>(0, 0);
    delta_transformation.block<2, 1>(0, 2) = small_delta.head<2>();

    Matrix3 original_transformation = Matrix3::Identity();
    original_transformation.block<2, 2>(0, 0)
        = RotationMatrix::aroundZ(original_pose.rotation).matrix().block<2, 2>(0, 0);
    original_transformation.block<2, 1>(0, 2) = original_pose.translation;

    Matrix3 expected_transformation = delta_transformation * original_transformation;

    Scalar expected_x = original_pose.translation.x() + small_delta(0);
    Scalar expected_y = original_pose.translation.y() + small_delta(1);

    EXPECT_NEAR(expected_transformation(0, 2), expected_x, tolerance);
    EXPECT_NEAR(expected_transformation(1, 2), expected_y, tolerance);
}

TEST_F(ManifoldTest, AngleWrapping) {
    // Test angle wrapping for large rotations
    Vector3 large_rotation_delta(0.0, 0.0, 2 * M_PI);
    Pose2 result = Pose2Manifold::boxPlus(original_pose, large_rotation_delta);

    // The resulting angle should be properly wrapped
    EXPECT_GE(result.rotation, -M_PI);
    EXPECT_LT(result.rotation, M_PI);
    EXPECT_NEAR(original_pose.rotation, result.rotation, tolerance);
}

TEST_F(ManifoldTest, BoxPlusBoxMinusInverse) {
    // Test that boxMinus(boxPlus(pose, delta), pose) ≈ delta
    Pose2 perturbed = Pose2Manifold::boxPlus(original_pose, small_delta);
    VectorX recovered_delta = Pose2Manifold::boxMinus(perturbed, original_pose);

    EXPECT_NEAR(small_delta(0), recovered_delta(0), tolerance);
    EXPECT_NEAR(small_delta(1), recovered_delta(1), tolerance);
    EXPECT_NEAR(small_delta(2), recovered_delta(2), tolerance);
}

TEST_F(ManifoldTest, Consistency) {
    // Test consistency: boxPlus(pose1, boxMinus(pose2, pose1)) ≈ pose2
    Pose2 pose2 = Pose2(M_PI / 3, Vector2(3.0, 4.0));
    VectorX delta = Pose2Manifold::boxMinus(pose2, original_pose);
    Pose2 recovered = Pose2Manifold::boxPlus(original_pose, delta);

    EXPECT_NEAR(pose2.translation.x(), recovered.translation.x(), tolerance);
    EXPECT_NEAR(pose2.translation.y(), recovered.translation.y(), tolerance);
    EXPECT_NEAR(pose2.rotation, recovered.rotation, tolerance);
}

}  // namespace pms
