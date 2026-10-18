#include "arm/kinematics.hpp"
#include <cmath>

namespace arm
{
    position forward(arm_angles angles, arm_constants constants)
    {
        position new_position;

        // calculate x and y from only x and y plane reference point
        double x = constants.length1 * cos(angles.shoulder) + constants.length2 * cos(angles.shoulder + angles.elbow) + 
        constants.length3 * cos(angles.shoulder + angles.elbow + angles.wrist);
        new_position.y = constants.length1 * sin(angles.shoulder) + constants.length2 * sin(angles.shoulder + angles.elbow) + 
        constants.length3 * sin(angles.shoulder + angles.elbow + angles.wrist) + constants.shoulder_height;

        // calculate x and z from world reference point, y remains the same
        new_position.z = x * sin(angles.base);
        new_position.x = x * cos(angles.base);

        return new_position;
    }
}