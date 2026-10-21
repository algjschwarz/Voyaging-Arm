#include "arm/motion.hpp"
#include <algorithm>

namespace arm
{
    namespace
    {
        double step(double current, double target, double max_step)
        {
            return current + std::clamp(target - current, -max_step, max_step);
        }
    }

    arm_angles step_toward(const arm_angles& current, const arm_angles& target, double max_rad_per_s, double dt_s)
    {
        const double s = max_rad_per_s * dt_s;
        return {
            .base = step(current.base, target.base, s),
            .shoulder = step(current.shoulder, target.shoulder, s),
            .elbow = step(current.elbow, target.elbow, s),
            .wrist = step(current.wrist, target.wrist, s),
            .wrist_spin = step(current.wrist_spin, target.wrist_spin, s),
            .gripper = step(current.gripper, target.gripper, s),
        };
    }
}
