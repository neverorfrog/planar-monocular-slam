#include <gtest/gtest.h>

#include "pms/camera/camera.h"
#include "pms/math/pose3.h"

namespace pms {

class CameraTest : public ::testing::Test {
   protected:
    void SetUp() override {
        // Create test camera
        camera.camera_matrix << 180, 0, 320, 0, 180, 240, 0, 0, 1;
        Matrix3 rotation = Matrix3::Identity();
        rotation << 0, 0, 1, -1, 0, 0, 0, -1, 0;  // Rotate camera to face forward in x direction
        Vector3 translation(0.2, 0.0, 0.0);  // Camera offset in robot frame
        camera.pose = Pose3(rotation, translation);
        camera.width = 640;
        camera.height = 480;
        camera.z_near = 0.0;
        camera.z_far = 5.0;

        // Test pose (robot at 1 meter on the x axis)
        test_pose = Pose2(0.0, Vector2(1.0, 0.0));

        // Test pose (robot one meter forward)
        test_pose_next = Pose2(0.0, Vector2(2.0, 0.0));

        // Test landmark in world frame
        test_landmark = Vector3(2.0, 0.0, 0.0);

        // Expected measurement (should be at image center for this setup)
        expected_measurement = Vector2(320, 240);
    }

    Camera camera;
    Pose2 test_pose;
    Pose2 test_pose_next;
    Vector3 test_landmark;
    Vector2 expected_measurement;
    const double tolerance = 1e-4;
};

TEST_F(CameraTest, Frame) {
    Pose3 camera_T_world = camera.computeWorldToCamera(Pose3(test_pose));
    Pose3 world_T_camera = camera_T_world.inverse();
    Pose3 robot_T_camera = camera.pose;
    Pose3 camera_T_robot = robot_T_camera.inverse();

    Pose3 world_T_robot = world_T_camera * camera_T_robot;
    Pose3 robot_T_world = world_T_robot.inverse();
    Vector3 landmark_in_robot = robot_T_world * test_landmark;
    EXPECT_EQ(landmark_in_robot.x(), 1.0);
    EXPECT_EQ(landmark_in_robot.y(), 0.0);
    EXPECT_EQ(landmark_in_robot.z(), 0.0);
}

TEST_F(CameraTest, Projection) {
    Pose3 camera_T_world = camera.computeWorldToCamera(Pose3(test_pose));
    Eigen::Matrix<Scalar, 3, 4> projection_matrix = camera.computeProjectionMatrix(camera_T_world);

    Vector3 point_in_camera = camera.pointInCamera(test_landmark, projection_matrix);
    Vector2 measurement = Vector2(point_in_camera.x() / point_in_camera.z(), point_in_camera.y() / point_in_camera.z());

    EXPECT_NEAR(measurement.x(), expected_measurement.x(), tolerance);
    EXPECT_NEAR(measurement.y(), expected_measurement.y(), tolerance);
    EXPECT_NEAR(point_in_camera.z(), (test_landmark.x() - test_pose.translation.x()) - camera.pose.translation.x(), tolerance);
}

}  // namespace pms
