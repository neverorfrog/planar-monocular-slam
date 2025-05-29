#include "pms/optimization/state.h"
#include <cstddef>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include "pms/optimization/manifold.h"

namespace pms {

State::State(const std::vector<Pose2>& initial_poses, const std::vector<Landmark>& initial_landmarks)
    : robot_poses(initial_poses), landmarks(initial_landmarks) {}

size_t State::getNumPoses() const {
    return robot_poses.size();
}

size_t State::getNumLandmarks() const {
    return landmarks.size();
}

int State::getDimension() const {
    return 3 * getNumPoses() + 3 * getNumLandmarks();
}

void State::applyIncrement(const VectorX& delta) {
    if (delta.size() != getDimension()) {
        throw std::invalid_argument("Delta size does not match state dimension");
    }

    // Apply pose increments
    constexpr int pose_dim = 3;  // SE(2) has 3 DOF
    for(size_t i = 0; i < robot_poses.size(); ++i) {
        int perturbation_index = i * pose_dim;
        Vector3 perturbation = delta.segment(perturbation_index, pose_dim);
        Pose2 perturbed = Pose2Manifold::boxPlus(robot_poses[i], perturbation);
        robot_poses[i] = perturbed;
    }

    for(size_t i = 0; i < landmarks.size(); ++i) {
        int perturbation_index = 3 * (i + robot_poses.size());
        Vector3 perturbation = delta.segment(perturbation_index, 3);
        Vector3 perturbed = landmarks[i].position + perturbation;
        landmarks[i].position = perturbed;
    }
}

std::vector<Vector3> State::getLandmarkPositions() const {
    std::vector<Vector3> positions;
    for (const auto& landmark : landmarks) {
        positions.push_back(landmark.position);
    }
    return positions;
}

}  // namespace pms
