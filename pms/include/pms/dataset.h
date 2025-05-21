#pragma once

#include <string>
#include <vector>
#include "pms/math/pose2.h"
#include "pms/math/pose3.h"
#include "pms/math/definitions.h"


namespace pms {


struct Camera {
    Matrix3 camera_matrix;
    Pose3 pose;
    Scalar z_near;
    Scalar z_far;
    int width;
    int height;
};

struct Landmark {
    Vector3 position;
    int id;

    Landmark() : id(0) {}
    Landmark(const Vector3& pos, int landmark_id) : position(pos), id(landmark_id) {
        if (pos.size() != 3) {
            throw std::invalid_argument("Landmark position must be a 3D vector.");
        }
    }
};

struct Measurement {
    int seq_number;
    int current_id;
    int actual_id;
    Vector2 image_point;

    Measurement() : seq_number(0), current_id(0), actual_id(0) {}
    Measurement(int seq, int cur_id, int act_id, const Vector2& img_pt)
        : seq_number(seq), current_id(cur_id), actual_id(act_id), image_point(img_pt) {
        if (img_pt.size() != 2) {
            throw std::invalid_argument("Measurement image_point must be a 2D vector.");
        }
    }
};

struct TrajPoint {
    int id;
    Pose2 odometry;
    Pose2 ground_truth;
    std::vector<Measurement> measurements;

    TrajPoint() : id(0) {}
    TrajPoint(int point_id, const Pose2& odom, const Pose2& gt)
        : id(point_id), odometry(odom), ground_truth(gt) {}
};

struct Dataset {
    Camera camera;
    std::vector<Landmark> world;
    std::vector<TrajPoint> trajectory;
};

// Function declarations
std::vector<TrajPoint> load_trajectory(const std::string& folderpath);
Camera load_camera_data(const std::string& folderpath);
std::vector<Landmark> load_world_data(const std::string& folderpath);
Dataset load_dataset(const std::string& folderpath);

// Example main function declaration (implementation in .cpp)
void run_dataset_loading_example(const std::string& base_data_path);

} // namespace pms