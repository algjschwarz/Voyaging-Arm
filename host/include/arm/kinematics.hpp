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

    struct arm_constants
    {
        int shoulder_height;
        int length1;
        int length2;
        int length3;
    };

    position forward(arm_angles input_angles, arm_constants constants);
    // wrist_angle is in radians, calculates values assuming link 3 is the end of the arm 
    arm_angles inverse(position input_position, arm_constants constants, double wrist_angle);

    // allow `std::cout << angles` / `std::cout << pos`
    std::ostream& operator<<(std::ostream& os, const arm_angles& angles);
    std::ostream& operator<<(std::ostream& os, const position& pos);
}