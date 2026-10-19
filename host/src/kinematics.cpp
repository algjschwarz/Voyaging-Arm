#include "arm/kinematics.hpp"
#include <cmath>
#include <iomanip>

namespace arm
{
    position forward(arm_angles input_angles, arm_constants constants)
    {
        position new_position;

        // calculate x and y from only x and y plane reference point
        double x = constants.length1 * cos(input_angles.shoulder) + constants.length2 * cos(input_angles.shoulder + input_angles.elbow) + 
        constants.length3 * cos(input_angles.shoulder + input_angles.elbow + input_angles.wrist);
        new_position.y = constants.length1 * sin(input_angles.shoulder) + constants.length2 * sin(input_angles.shoulder + input_angles.elbow) + 
        constants.length3 * sin(input_angles.shoulder + input_angles.elbow + input_angles.wrist) + constants.shoulder_height;

        // calculate x and z from world reference point, y remains the same
        new_position.z = x * sin(input_angles.base);
        new_position.x = x * cos(input_angles.base);

        return new_position;
    }

    arm_angles inverse(position input_position, arm_constants constants, double wrist_angle)
    {
        arm_angles calculated_angles;
        // the angles are calculated in 2 phases
        // Phase 1: the x-z plane
        // Phase 2: the x-y plane

        // Phase 1: calculated in the x-z plane
        calculated_angles.base = atan2(input_position.z, input_position.x);
        double link1_to_link3 = sqrt(pow(input_position.z, 2) + pow(input_position.x, 2));

        // Phase 2: calculated in the x-y plane
        double link1_to_link2_x = link1_to_link3 - constants.length3 * cos(wrist_angle); 
        double link1_to_link2_y = input_position.y - constants.shoulder_height - constants.length3 * sin(wrist_angle); 
        double link1_to_link3_squared = pow(link1_to_link2_x, 2) + pow(link1_to_link2_y, 2);

        calculated_angles.elbow = acos((link1_to_link3_squared - pow(constants.length1, 2) - pow(constants.length2, 2)) / (2 * constants.length1 * constants.length2));
        calculated_angles.shoulder = atan2(link1_to_link2_y, link1_to_link2_x) - 
        atan2(constants.length2 * sin(calculated_angles.elbow), constants.length1 + constants.length2 * cos(calculated_angles.elbow));
        calculated_angles.wrist = wrist_angle - calculated_angles.shoulder - calculated_angles.elbow;
        return calculated_angles;
    }

    std::ostream& operator<<(std::ostream& os, const arm_angles& angles)
    {
        std::ios old_state(nullptr);
        old_state.copyfmt(os);
        os << std::fixed << std::setprecision(4)
           << "arm_angles(rad){ base: " << angles.base
           << ", shoulder: " << angles.shoulder
           << ", elbow: " << angles.elbow
           << ", wrist: " << angles.wrist
           << ", wrist_spin: " << angles.wrist_spin
           << ", gripper: " << angles.gripper << " }";
        os.copyfmt(old_state);
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const position& pos)
    {
        std::ios old_state(nullptr);
        old_state.copyfmt(os);
        os << std::fixed << std::setprecision(4)
           << "position{ x: " << pos.x
           << ", y: " << pos.y
           << ", z: " << pos.z << " }";
        os.copyfmt(old_state);
        return os;
    }
}
