#include "pms/optimization/state.h"
#include <cstddef>
#include "pms/optimization/manifold.h"

namespace pms {

State::State(const std::vector<Pose2>& initial_poses, const std::vector<Landmark>& initial_landmarks)
    : robot_poses(initial_poses), landmarks(initial_landmarks) {}

size_t State::getNumPoses() const {
    return robot_poses.size();
}

size_t State::getNumLandmarks() const {
    return std::count_if(landmarks.begin(), landmarks.end(),
                         [](const Landmark& landmark) { return landmark.valid; });
}

int State::getStateDimension() const {
    return 3 * getNumPoses() + 3 * getNumLandmarks();
}

void State::applyIncrement(const VectorX& delta) {
    if (delta.size() != getStateDimension()) {
        throw std::invalid_argument("Delta size does not match state dimension");
    }

    for(size_t i = 0; i < robot_poses.size(); ++i) {
        int perturbation_index = 0; // TODO: Define mapping
        Vector3 perturbation = delta.segment(perturbation_index, 3);
        Pose2 perturbed = Pose2Manifold::boxPlus(robot_poses[i], perturbation);
        robot_poses[i] = perturbed;
    }

    for(size_t i = 0; i < landmarks.size(); ++i) {
        int perturbation_index = 0; // TODO: Define mapping
        Vector3 perturbation = delta.segment(perturbation_index, 3);
        Vector3 perturbed = landmarks[i].position + perturbation;
        landmarks[i].position = perturbed;
    }
}

}  // namespace pms
