#include "pms/optimization/state.h"

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
    // TODO: Apply increment using manifold operations (boxPlus)
}

}  // namespace pms
