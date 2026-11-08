// arm_simulation: the simulation on its own. Nothing is sent to the real arm.
// When the arm is calibrated, the sliders stop where its servos stop, same as in `arm`.
#include "arm/limits.hpp"
#include "arm/servo_map.hpp"
#include "arm/simulation.hpp"

#include <cstdio>
#include <exception>

#ifndef ARM_CALIBRATION_FILE
#define ARM_CALIBRATION_FILE "servos.txt"
#endif

int main() {
    arm::arm_limits limits = arm::VOYAGING_ARM_LIMITS;
    try {
        limits = arm::within_servo_travel(limits, arm::load_calibration(ARM_CALIBRATION_FILE));
    } catch (const std::exception& e) {
        std::fprintf(stderr, "using the CAD limits only (%s)\n", e.what());
    }
    arm::run_simulation({}, limits);
}
