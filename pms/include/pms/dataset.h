#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "pms/math/definitions.h"
#include "pms/math/pose2.h"
#include "pms/math/pose3.h"

namespace fs = std::filesystem;

namespace pms {

struct Camera {
    Matrix3 camera_matrix;
    Pose3 pose;
    Scalar z_near;
    Scalar z_far;
    int width;
    int height;

    Camera() : z_near(0), z_far(0), width(0), height(0) {}

    Camera(const Camera& other)
        : camera_matrix(other.camera_matrix),
          pose(other.pose),
          z_near(other.z_near),
          z_far(other.z_far),
          width(other.width),
          height(other.height) {}

    const Camera& operator=(const Camera& other) {
        if (this != &other) {
            camera_matrix = other.camera_matrix;
            pose = other.pose;
            z_near = other.z_near;
            z_far = other.z_far;
            width = other.width;
            height = other.height;
        }
        return *this;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Camera:\n"
            << "Camera Matrix:\n" << camera_matrix << "\n"
            << "Pose:\n" << pose.toString() << "\n"
            << "z_near: " << z_near << "\n"
            << "z_far: " << z_far << "\n"
            << "Width: " << width << "\n"
            << "Height: " << height;
        return oss.str();
    }
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

    Landmark(const Landmark& other) : position(other.position), id(other.id) {}

    const Landmark& operator=(const Landmark& other) {
        if (this != &other) {
            position = other.position;
            id = other.id;
        }
        return *this;
    }

    bool operator==(const Landmark& other) const {
        return id == other.id && position.isApprox(other.position);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Landmark ID: " << id << "\n"
            << "Position: " << position.transpose();
        return oss.str();
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

    Measurement(const Measurement& other)
        : seq_number(other.seq_number),
          current_id(other.current_id),
          actual_id(other.actual_id),
          image_point(other.image_point) {}

    const Measurement& operator=(const Measurement& other) {
        if (this != &other) {
            seq_number = other.seq_number;
            current_id = other.current_id;
            actual_id = other.actual_id;
            image_point = other.image_point;
        }
        return *this;
    }

    bool operator==(const Measurement& other) const {
        return seq_number == other.seq_number && current_id == other.current_id
               && actual_id == other.actual_id && image_point.isApprox(other.image_point);
    }

    bool isNear(const Measurement& other, Scalar threshold) const {
        return (image_point - other.image_point).norm() < threshold;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Measurement:\n"
            << "Sequence Number: " << seq_number << "\n"
            << "Current ID: " << current_id << "\n"
            << "Actual ID: " << actual_id << "\n"
            << "Image Point: " << image_point.transpose();
        return oss.str();
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

    TrajPoint(const TrajPoint& other)
        : id(other.id),
          odometry(other.odometry),
          ground_truth(other.ground_truth),
          measurements(other.measurements) {}

    const TrajPoint& operator=(const TrajPoint& other) {
        if (this != &other) {
            id = other.id;
            odometry = other.odometry;
            ground_truth = other.ground_truth;
            measurements = other.measurements;
        }
        return *this;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Trajectory Point ID: " << id << "\n"
            << "Odometry: " << odometry.toString() << "\n"
            << "Ground Truth: " << ground_truth.toString() << "\n"
            << "Measurements:\n";
        for (const auto& meas : measurements) {
            oss << meas.toString() << "\n";
        }
        return oss.str();
    }
};

// Function declarations
std::vector<TrajPoint> load_trajectory(const std::string& folderpath);
Camera load_camera_data(const std::string& folderpath);
std::vector<Landmark> load_world_data(const std::string& folderpath);

struct Dataset {
    Camera camera;
    std::vector<Landmark> world;
    std::vector<TrajPoint> trajectory;

    Dataset(const std::string& folderpath) {
        if (!fs::exists(folderpath) || !fs::is_directory(folderpath)) {
            throw std::runtime_error("Dataset folder not found or is not a directory: " + folderpath);
        }

        camera = load_camera_data(folderpath);
        world = load_world_data(folderpath);
        trajectory = load_trajectory(folderpath);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Dataset:\n"
            << camera.toString() << "\n"
            << "World:\n";
        for (const auto& landmark : world) {
            oss << landmark.toString() << "\n";
        }
        oss << "Trajectory:\n";
        for (const auto& traj_point : trajectory) {
            oss << traj_point.toString() << "\n";
        }
        return oss.str();
    }
};

}  // namespace pms