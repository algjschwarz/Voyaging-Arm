#include <iostream>
#include "arm/presets.hpp"

int main() {
    arm::arm_angles angles = {3.14/2, 0, 0, 0, 0, 0};
    arm::position position = arm::forward(angles, arm::VOYAGING_ARM);
    std::cout << position << std::endl;
}
