#include <gtest/gtest.h>
#include <stdexcept>

#include "pms/dataset/landmark.h"
#include "pms/math/pose2.h"
#include "pms/optimization/state.h"

namespace pms {

class StateTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Create a state with 3 poses and 2 landmarks
        robot_poses.push_back(Pose2(0.0, Vector2(0.0, 0.0)));       // Identity pose
        robot_poses.push_back(Pose2(M_PI / 2, Vector2(1.0, 1.0)));  // 90 degrees, moved in x and y

        landmarks.push_back(Landmark(Vector3(2.0, 2.0, 1.0), 0, true));   // Landmark 0
        landmarks.push_back(Landmark(Vector3(4.0, 0.0, 1.0), 2, false));  // Landmark 1 (invalid)
        landmarks.push_back(Landmark(Vector3(3.0, 1.0, 1.0), 1, true));   // Landmark 2

        state = State(robot_poses, landmarks);
    }

    std::vector<Pose2> robot_poses;
    std::vector<Landmark> landmarks;
    State state;
    const double tol = 1e-6;
};

TEST_F(StateTest, DISABLED_IncrementDimension) {
    // Check the dimension of the state
    size_t expected_dimension = 9;
    EXPECT_EQ(state.getDimension(), expected_dimension);
}

TEST_F(StateTest, DISABLED_ApplyIncrementToStateVector) {
    // Create increment vector with small changes
    VectorX increment = VectorX::Zero(12);

    // Small increments for poses (2 poses * 3 DOF = 6 elements)
    increment(0) = 0.1;   // pose 0 dx
    increment(1) = 0.2;   // pose 0 dy
    increment(2) = 0.05;  // pose 0 dtheta

    increment(3) = 0.0;  // pose 1 dx
    increment(4) = 0.0;  // pose 1 dy
    increment(5) = 0.0;   // pose 1 dtheta

    // Small increments for landmarks (2 landmarks * 3 DOF = 6 elements)
    increment(6) = 0.1;   // landmark 0 dx
    increment(7) = 0.2;   // landmark 0 dy
    increment(8) = 0.04;  // landmark 0 dz

    increment(9) = 0.1;   // landmark 2 dx (1 is invalid)
    increment(10) = 0.1;  // landmark 2 dy (1 is invalid)
    increment(11) = 0.1;  // landmark 2 dz (1 is invalid)

    // Store original state
    State original_state{state};

    // Apply increment
    EXPECT_NO_THROW(state.applyIncrement(increment));

    // Verify manually that the poses and landmarks were updated correctly
    EXPECT_NEAR(state.robot_poses[0].translation.x(), original_state.robot_poses[0].translation.x() + 0.1, tol);
    EXPECT_NEAR(state.robot_poses[0].translation.y(), original_state.robot_poses[0].translation.y() + 0.2, tol);
    EXPECT_NEAR(state.robot_poses[0].rotation, original_state.robot_poses[0].rotation + 0.05, tol);

    EXPECT_NEAR(state.robot_poses[1].translation.x(), original_state.robot_poses[1].translation.x(), tol);
    EXPECT_NEAR(state.robot_poses[1].translation.y(), original_state.robot_poses[1].translation.y(), tol);
    EXPECT_NEAR(state.robot_poses[1].rotation, original_state.robot_poses[1].rotation, tol);

    EXPECT_NEAR(state.landmarks[0].position.x(), original_state.landmarks[0].position.x() + 0.1, tol);
    EXPECT_NEAR(state.landmarks[0].position.y(), original_state.landmarks[0].position.y() + 0.2, tol);
    EXPECT_NEAR(state.landmarks[0].position.z(), original_state.landmarks[0].position.z() + 0.04, tol);

    EXPECT_NEAR(state.landmarks[1].position.x(), original_state.landmarks[1].position.x(), tol);
    EXPECT_NEAR(state.landmarks[1].position.y(), original_state.landmarks[1].position.y(), tol);
    EXPECT_NEAR(state.landmarks[1].position.z(), original_state.landmarks[1].position.z(), tol);

    EXPECT_NEAR(state.landmarks[2].position.x(), original_state.landmarks[2].position.x() + 0.1 , tol);
    EXPECT_NEAR(state.landmarks[2].position.y(), original_state.landmarks[2].position.y() + 0.1 , tol);
    EXPECT_NEAR(state.landmarks[2].position.z(), original_state.landmarks[2].position.z() + 0.1 , tol);
}

TEST_F(StateTest, IncrementDimensionMismatch) {
    // Test with wrong dimension increment
    VectorX wrong_increment = VectorX::Zero(10);  // Should be 12

    // This should either throw an exception or handle gracefully
    // The exact behavior depends on implementation
    EXPECT_THROW(state.applyIncrement(wrong_increment), std::invalid_argument);
}

}  // namespace pms
