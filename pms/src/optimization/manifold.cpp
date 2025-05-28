#include "pms/optimization/manifold.h"

#include <cassert>

namespace pms {

// Pose2Manifold Implementation
Pose2 Pose2Manifold::boxPlus(const Pose2& state, const VectorX& delta) {
    assert(delta.size() == 3 && "Delta must be exactly size 3 for Pose2 boxPlus");

    // Apply updates in global coordinates for bundle adjustment
    Pose2 result = state;
    result.translation += Vector2(delta(0), delta(1));  // Direct addition in global frame
    result.rotation += Angle(delta(2));                 // Rotation addition
    result.rotation.normalize();
    return result;
}

VectorX Pose2Manifold::boxMinus(const Pose2& state1, const Pose2& state2) {
    VectorX delta(3);
    // Direct difference in global coordinates for bundle adjustment
    delta(0) = state1.translation.x() - state2.translation.x();  // dx
    delta(1) = state1.translation.y() - state2.translation.y();  // dy
    delta(2) = state1.rotation - state2.rotation;                // dtheta

    // Handle angle wraparound
    delta(2) = Angle::normalize(delta(2));

    return delta;
}

}  // namespace pms
