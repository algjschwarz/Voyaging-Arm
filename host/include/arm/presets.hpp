#pragma once
#include "arm/kinematics.hpp"
#include "arm/geometry.hpp"

namespace arm {
    // The arm as built, straight from CAD. length3 ends at the tips of the closed jaws.
    constexpr arm_constants VOYAGING_ARM{
        .shoulder_height = geometry::shoulder_height,
        .length1 = geometry::link1,
        .length2 = geometry::link2,
        .length3 = geometry::link3,
        .shoulder_offset = geometry::shoulder_offset,
        .wrist_offset = geometry::wrist_offset,
    };

    // Same arm with the target point moved along link 3. Negative pulls it back into the claw.
    constexpr arm_constants with_reach(arm_constants c, double extra_mm) {
        c.length3 += extra_mm;
        return c;
    }
}
