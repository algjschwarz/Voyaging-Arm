#pragma once
#include "arm/kinematics.hpp"

namespace arm {
    // Moves every joint of `current` toward `target` by at most max_rad_per_s * dt_s, and returns the result.
    // Call it once per frame: the returned angles are the in-between pose to draw and to send to the servos.
    // Each joint moves independently, so a joint with less distance to cover arrives first.
    arm_angles step_toward(const arm_angles& current, const arm_angles& target, double max_rad_per_s, double dt_s);
}
