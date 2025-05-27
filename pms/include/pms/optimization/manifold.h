#pragma once

#include "pms/math/definitions.h"
#include "pms/math/pose2.h"
#include "pms/math/pose3.h"

namespace pms {

/**
 * @brief SE(2) manifold operations for planar robot poses
 * 
 * Provides static functions for manifold operations on 2D poses with proper handling
 * of rotation angle wraparound and composition.
 */
class Pose2Manifold {
public:
    /**
     * @brief Manifold plus operation: state ⊞ delta
     * @param state Current state on the manifold
     * @param delta Tangent space perturbation (3D: [dx, dy, dtheta])
     * @return Updated state after applying perturbation
     */
    static Pose2 boxPlus(const Pose2& state, const VectorX& delta);

    /**
     * @brief Manifold minus operation: state1 ⊟ state2
     * @param state1 First state on the manifold
     * @param state2 Second state on the manifold
     * @return Tangent space difference (3D: [dx, dy, dtheta])
     */
    static VectorX boxMinus(const Pose2& state1, const Pose2& state2);

    /**
     * @brief Get the dimension of the tangent space
     * @return Dimension of the tangent space (always 3 for SE(2))
     */
    static constexpr int getDimension() { return 3; }
};

/**
 * @brief SE(3) manifold operations for 3D poses
 * 
 * Provides static functions for manifold operations on 3D poses with proper handling
 * of rotation matrix constraints.
 */
class Pose3Manifold {
public:
    /**
     * @brief Manifold plus operation: state ⊞ delta
     * @param state Current state on the manifold
     * @param delta Tangent space perturbation (6D: [dx, dy, dz, drx, dry, drz])
     * @return Updated state after applying perturbation
     */
    static Pose3 boxPlus(const Pose3& state, const VectorX& delta);

    /**
     * @brief Manifold minus operation: state1 ⊟ state2
     * @param state1 First state on the manifold
     * @param state2 Second state on the manifold
     * @return Tangent space difference (6D: [dx, dy, dz, drx, dry, drz])
     */
    static VectorX boxMinus(const Pose3& state1, const Pose3& state2);

    /**
     * @brief Get the dimension of the tangent space
     * @return Dimension of the tangent space (always 6 for SE(3))
     */
    static constexpr int getDimension() { return 6; }
};

class MeasurementManifold {
public:
    /**
     * @brief Manifold plus operation: state ⊞ delta
     * @param state Current measurement
     * @param delta Tangent space perturbation (2D: [du, dv])
     * @return Updated measurement after applying perturbation
     */
    static Vector2 boxPlus(const Vector2& state, const VectorX& delta) {
        return state + delta.head<2>();
    }

    /**
     * @brief Manifold minus operation: state1 ⊟ state2
     * @param state1 First measurement
     * @param state2 Second measurement
     * @return Tangent space difference (2D: [du, dv])
     */
    static VectorX boxMinus(const Vector2& state1, const Vector2& state2) {
        return state1 - state2;
    }

    /**
     * @brief Get the dimension of the tangent space
     * @return Dimension of the tangent space (always 2 for R²)
     */
    static constexpr int getDimension() { return 2; }
};

}  // namespace pms
