#pragma once

#include <cassert>
#include <iostream>

#include "pms/dataset.h"
#include "pms/solution.h"

namespace pms {

inline Vector3 triangulateLandmark(const std::vector<Pose2>& odom_poses, const std::vector<Measurement>& measurements, const Camera& camera) {
    // Placeholder for the actual triangulation logic
    // This function should return the 3D position of the landmark
    return Vector3::Zero();
}

inline void triangulate(Solution& solution, const Dataset& dataset) {

    // Parse measurement for each landmark
    std::vector<std::vector<Measurement>> measurements_per_landmark(dataset.world.size());
    for (const auto& traj_point : dataset.trajectory) {
        for (const auto& measurement : traj_point.measurements) {
            int landmark_id = measurement.landmark_id;
            assert(landmark_id >= 0 && landmark_id < dataset.world.size());
            measurements_per_landmark[landmark_id].push_back(measurement);
        }
    }

    for (size_t i = 0; i < measurements_per_landmark.size(); ++i) {
        std::cout << "Triangulating landmark " << i << " with " << measurements_per_landmark[i].size()
                  << " measurements." << std::endl;
        const std::vector<Measurement>& measurements = measurements_per_landmark[i];
        if (measurements.size() < 2) {
            std::cerr << "Not enough measurements to triangulate landmark " << i << std::endl;
            continue;
        }

        std::vector<Pose2> odom_poses;
        for (const auto& measurement : measurements) {
            int seq_number = measurement.seq_number;
            assert(seq_number >= 0 && seq_number < dataset.trajectory.size());
            odom_poses.push_back(dataset.trajectory[seq_number].odometry);
        }

        Vector3 landmark_pos_guess = triangulateLandmark(odom_poses, measurements, dataset.camera);
    }
}

}  // namespace pms
