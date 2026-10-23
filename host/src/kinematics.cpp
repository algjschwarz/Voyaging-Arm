#include "arm/kinematics.hpp"
#include <cmath>
#include <iomanip>

namespace arm
{
    position forward(arm_angles input_angles, arm_constants constants)
    {
        position new_position;

        // calculate x and z from only x and z plane reference point
        // link 3 runs wrist_offset to the side of the wrist joint, perpendicular to its own direction
        double wrist_angle = input_angles.shoulder + input_angles.elbow + input_angles.wrist;
        double x = constants.shoulder_offset + constants.length1 * cos(input_angles.shoulder) + constants.length2 * cos(input_angles.shoulder + input_angles.elbow) + 
        constants.length3 * cos(wrist_angle) - constants.wrist_offset * sin(wrist_angle);
        new_position.z = constants.length1 * sin(input_angles.shoulder) + constants.length2 * sin(input_angles.shoulder + input_angles.elbow) + 
        constants.length3 * sin(wrist_angle) + constants.wrist_offset * cos(wrist_angle) + constants.shoulder_height;

        // calculate x and y from world reference point, z remains the same
        new_position.y = x * sin(input_angles.base);
        new_position.x = x * cos(input_angles.base);

        return new_position;
    }

    arm_angles inverse(position input_position, arm_constants constants, double wrist_angle)
    {
        arm_angles calculated_angles;
        // the angles are calculated in 2 phases
        // Phase 1: the x-y plane
        // Phase 2: the x-z plane

        // Phase 1: calculated in the x-y plane
        calculated_angles.base = atan2(input_position.y, input_position.x);
        double link1_to_link3 = sqrt(pow(input_position.y, 2) + pow(input_position.x, 2)) - constants.shoulder_offset;

        // Phase 2: calculated in the x-z plane
        double link1_to_link2_x = link1_to_link3 - constants.length3 * cos(wrist_angle) + constants.wrist_offset * sin(wrist_angle); 
        double link1_to_link2_z = input_position.z - constants.shoulder_height - constants.length3 * sin(wrist_angle) - constants.wrist_offset * cos(wrist_angle); 
        double link1_to_link3_squared = pow(link1_to_link2_x, 2) + pow(link1_to_link2_z, 2);

        // positive for elbow down
        calculated_angles.elbow =  - acos((link1_to_link3_squared - pow(constants.length1, 2) - pow(constants.length2, 2)) / (2 * constants.length1 * constants.length2));
        calculated_angles.shoulder = atan2(link1_to_link2_z, link1_to_link2_x) - 
        atan2(constants.length2 * sin(calculated_angles.elbow), constants.length1 + constants.length2 * cos(calculated_angles.elbow));
        calculated_angles.wrist = wrist_angle - calculated_angles.shoulder - calculated_angles.elbow;
        return calculated_angles;
    }

    joint_chain joint_points(arm_angles a, arm_constants c)
    {
        const double se = a.shoulder + a.elbow;
        const double wa = se + a.wrist;

        // in the vertical plane through the arm: (r = reach, z = height)
        struct RZ { double r, z; };
        std::array<RZ, 6> rz{};
        rz[0] = {0.0, 0.0};
        rz[1] = {c.shoulder_offset, c.shoulder_height};
        rz[2] = {rz[1].r + c.length1 * cos(a.shoulder), rz[1].z + c.length1 * sin(a.shoulder)};
        rz[3] = {rz[2].r + c.length2 * cos(se),         rz[2].z + c.length2 * sin(se)};
        rz[4] = {rz[3].r - c.wrist_offset * sin(wa),    rz[3].z + c.wrist_offset * cos(wa)};
        rz[5] = {rz[4].r + c.length3 * cos(wa),         rz[4].z + c.length3 * sin(wa)};

        joint_chain out{};
        const double cb = cos(a.base), sb = sin(a.base);
        for (std::size_t i = 0; i < rz.size(); ++i) {
            out[i] = { rz[i].r * cb, rz[i].r * sb, rz[i].z };
        }
        return out;
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
