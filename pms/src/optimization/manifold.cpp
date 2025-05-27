#include "pms/optimization/manifold.h"
#include <cassert>

namespace pms {

// Pose2Manifold Implementation
Pose2 Pose2Manifold::boxPlus(const Pose2& state, const VectorX& delta) {
    assert(delta.size() == 3 && "Delta must be exactly size 3 for Pose2 boxPlus");
    const Pose2 delta_pose(delta(2), delta(0), delta(1));
    return state + delta_pose;
}

VectorX Pose2Manifold::boxMinus(const Pose2& state1, const Pose2& state2) {
    Pose2 delta_pose = state1 - state2;
    VectorX delta(3);
    delta(0) = delta_pose.translation.x();  // dx
    delta(1) = delta_pose.translation.y();  // dy
    delta(2) = delta_pose.rotation;  // dtheta
    return delta;
}

// Pose3Manifold Implementation
Pose3 Pose3Manifold::boxPlus(const Pose3& state, const VectorX& delta) {
    assert(delta.size() == 6 && "Delta must be exactly size 6 for Pose3 boxPlus");

    // Apply translation update (first 3 elements)
    Vector3 translation_delta = delta.head<3>();
    Pose3 result = state;
    result.translation += translation_delta;
    
    // Apply rotation update (last 3 elements) using small angle approximation
    Vector3 rotation_delta = delta.tail<3>();
    // For small angles, we can approximate the rotation update
    result.rotateX(rotation_delta(0));
    result.rotateY(rotation_delta(1));
    result.rotateZ(rotation_delta(2));
    
    return result;
}

VectorX Pose3Manifold::boxMinus(const Pose3& state1, const Pose3& state2) {
    VectorX delta(6);
    
    // Translation difference (first 3 elements)
    Vector3 translation_diff = state1.translation - state2.translation;
    delta.head<3>() = translation_diff;
    
    // Rotation difference (last 3 elements)
    // This is a simplified implementation - in practice you'd want to use
    // proper Lie algebra operations for SE(3)
    Vector3 rpy1 = state1.rotation.getRPY();
    Vector3 rpy2 = state2.rotation.getRPY();
    delta.tail<3>() = rpy1 - rpy2;
    
    return delta;
}

}  // namespace pms
