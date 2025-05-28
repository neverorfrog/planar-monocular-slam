#include <gtest/gtest.h>

#include "pms/dataset/landmark.h"
#include "pms/math/pose2.h"
#include "pms/optimization/manifold.h"
#include "pms/optimization/state.h"

namespace pms {

class StateTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Create a state with 3 poses and 2 landmarks
        robot_poses.push_back(Pose2(0.0, Vector2(0.0, 0.0)));       // Identity pose
        robot_poses.push_back(Pose2(M_PI / 4, Vector2(1.0, 0.0)));  // 45 degrees, moved in x
        robot_poses.push_back(Pose2(M_PI / 2, Vector2(1.0, 1.0)));  // 90 degrees, moved in x and y

        landmarks.push_back(Landmark(Vector3(2.0, 2.0, 1.0), 0, true));  // Landmark 0
        landmarks.push_back(Landmark(Vector3(3.0, 1.0, 1.0), 1, true));  // Landmark 1

        state = State(robot_poses, landmarks);
    }

    std::vector<Pose2> robot_poses;
    std::vector<Landmark> landmarks;
    State state;
    const double tolerance = 1e-6;
};

TEST_F(StateTest, ApplyIncrementToStateVector) {
    // Create increment vector with small changes
    VectorX increment = VectorX::Zero(15);

    // Small increments for poses (3 poses * 3 DOF = 9 elements)
    increment(0) = 0.1;   // pose 0 dx
    increment(1) = 0.2;   // pose 0 dy
    increment(2) = 0.05;  // pose 0 dtheta

    increment(3) = 0.15;  // pose 1 dx
    increment(4) = 0.25;  // pose 1 dy
    increment(5) = 0.1;   // pose 1 dtheta

    increment(6) = 0.2;   // pose 2 dx
    increment(7) = 0.3;   // pose 2 dy
    increment(8) = 0.15;  // pose 2 dtheta

    // Small increments for landmarks (2 landmarks * 3 DOF = 6 elements)
    increment(9) = 0.05;   // landmark 0 dx
    increment(10) = 0.1;   // landmark 0 dy
    increment(11) = 0.02;  // landmark 0 dz

    increment(12) = 0.08;  // landmark 1 dx
    increment(13) = 0.12;  // landmark 1 dy
    increment(14) = 0.03;  // landmark 1 dz

    // Store original state
    State original_state{state};

    // Apply increment
    state.applyIncrement(increment);

    // Verify the pose update was done correctly by manually applying boxPlus
    Pose2 expected_pose_0 = Pose2Manifold::boxPlus(original_state.robot_poses[0], increment.segment<3>(0));
    EXPECT_NEAR(expected_pose_0.translation.x(), state.robot_poses[0].translation.x(), tolerance);
    EXPECT_NEAR(expected_pose_0.translation.y(), state.robot_poses[0].translation.y(), tolerance);
    EXPECT_NEAR(expected_pose_0.rotation, state.robot_poses[0].rotation, tolerance);

    // Verify another pose update
    Pose2 expected_pose_1 = Pose2Manifold::boxPlus(original_state.robot_poses[1], increment.segment<3>(3));
    EXPECT_NEAR(expected_pose_1.translation.x(), state.robot_poses[1].translation.x(), tolerance);
    EXPECT_NEAR(expected_pose_1.translation.y(), state.robot_poses[1].translation.y(), tolerance);
    EXPECT_NEAR(expected_pose_1.rotation, state.robot_poses[1].rotation, tolerance);
}

TEST_F(StateTest, IncrementDimensionMismatch) {
    // Test with wrong dimension increment
    VectorX wrong_increment = VectorX::Zero(10);  // Should be 15

    // This should either throw an exception or handle gracefully
    // The exact behavior depends on implementation
    EXPECT_DEATH(state.applyIncrement(wrong_increment), ".*");
}

}  // namespace pms
