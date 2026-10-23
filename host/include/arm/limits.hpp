#pragma once
#include "arm/kinematics.hpp"
#include <numbers>
#include <string>

namespace arm {
    // How far one joint may turn, in radians, in the kinematics convention (see kinematics.hpp).
    struct joint_range
    {
        double min = 0;
        double max = 0;
    };

    // Where the arm can go without hurting itself.
    struct arm_limits
    {
        joint_range base;
        joint_range shoulder;
        joint_range elbow;
        joint_range wrist;
        double ground_clearance = 0;  // mm every joint and the tip must stay above the table (z = 0)
    };

    namespace detail {
        constexpr double deg(double d) { return d * std::numbers::pi / 180.0; }
    }

    // The arm as built. Pitch joints: where the parts first collide in cad/arm/Arm.FCStd (each joint
    // swept on its own, 0.5 deg steps), minus a 5 deg margin. Measured 2026-10-23; redo after changing the CAD.
    constexpr arm_limits VOYAGING_ARM_LIMITS{
        .base = {detail::deg(-90), detail::deg(82)},        // no collision; the calibrated servo travel (cmd 0..180)
        .shoulder = {detail::deg(-26), detail::deg(139)},   // ArmLink hits the Rotator at -31.5 / +144.5
        .elbow = {detail::deg(-135), detail::deg(135)},     // UpperArm hits ArmLink at +-140.5
        .wrist = {detail::deg(-122), detail::deg(121)},     // WristLink hits UpperArm at -127.1 / +126.9
        .ground_clearance = 10.0,
    };

    // Why `pose` would hurt the arm, e.g. "elbow past -120 deg" or "tip 6 mm above the table".
    // Empty when the pose is safe.
    std::string why_unsafe(const arm_angles& pose, const arm_constants& constants, const arm_limits& limits);
}
