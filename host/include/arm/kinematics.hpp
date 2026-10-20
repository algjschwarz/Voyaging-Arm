#pragma once
#include <ostream>

namespace arm{
    // includes all angles for servos, stored in radians
    struct arm_angles
    {
        double base = 0;
        double shoulder = 0;
        double elbow = 0;
        double wrist = 0;
        double wrist_spin = 0;
        double gripper = 0;
    };

    struct position
    {
        double x = 0;
        double y = 0;
        double z = 0;
    };

    // lengths in mm, from CAD via presets.hpp / geometry.hpp. z is up, base yaw is atan2(y, x).
    // Each joint matches a named coordinate system in cad/arm/Arm.FCStd (rotator_shoulder,
    // link_far_joint, wrist_near_joint, wrist_roll).
    struct arm_constants
    {
        double shoulder_height;   // base (z = 0) to the shoulder axis
        double length1;           // shoulder axis to elbow axis
        double length2;           // elbow axis to wrist pitch axis
        double length3;           // along the roll axis to the jaw tips
        double shoulder_offset;   // yaw axis to shoulder axis, horizontal, in the reach direction
        double wrist_offset;      // wrist pitch axis to roll axis, perpendicular to link 3
    };

    position forward(arm_angles input_angles, arm_constants constants);
    // wrist_angle is in radians, calculates values assuming link 3 is the end of the arm 
    arm_angles inverse(position input_position, arm_constants constants, double wrist_angle);

    // allow `std::cout << angles` / `std::cout << pos`
    std::ostream& operator<<(std::ostream& os, const arm_angles& angles);
    std::ostream& operator<<(std::ostream& os, const position& pos);
}
