#pragma once

#include "pms/dataset.h"
#include "pms/math/definitions.h"
#include "pms/math/pose2.h"

namespace pms {

struct Solution {
    std::vector<Pose2> trajectory;
    std::vector<Landmark> world;
    std::vector<Scalar> errors;

    Solution() = default;

    Solution(const Dataset& dataset) {
        for (const auto& traj_point : dataset.trajectory) {
            trajectory.push_back(traj_point.odometry);
        }
        for (const auto& landmark : dataset.world) {
            world.push_back(Landmark(landmark.id));
        }
    }

    Solution(const Solution& other)
        : trajectory(other.trajectory), world(other.world), errors(other.errors) {}

    const Solution& operator=(const Solution& other) {
        if (this != &other) {
            trajectory = other.trajectory;
            world = other.world;
            errors = other.errors;
        }
        return *this;
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "Solution:\n"
            << "Trajectory:\n";
        for (const auto& pose : trajectory) {
            oss << pose.toString() << "\n";
        }
        oss << "World:\n";
        for (const auto& landmark : world) {
            oss << landmark.toString() << "\n";
        }
        oss << "Errors:\n";
        for (const auto& error : errors) {
            oss << error << "\n";
        }
        return oss.str();
    }
};

}  // namespace pms