#include <iostream>
#include "arm/kinematics.hpp"

int main() {
    arm::arm_constants constants = {.shoulder_height = 5, .length1 = 10, .length2 = 10, .length3 = 5};
    arm::arm_angles angles = {3.14/2, 0, 0, 0, 0, 0};
    arm::position position = arm::forward(angles, constants);
    std::cout << "(" << position.x << "," << position.y << "," << position.z << ")" << "\n";
}