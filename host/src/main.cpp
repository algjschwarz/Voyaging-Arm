// arm: drives the real arm from the viewer.
//   arm /dev/ttyACM0
// Every servo must be calibrated first (arm_calibrate). For the simulation alone, run arm_viewer.
//
// The viewer hands over the pose every frame; this sends it to the Arduino 50 times a second.

#include "arm/serial_link.hpp"
#include "arm/servo_map.hpp"
#include "arm/viewer.hpp"

#include <cstdio>
#include <exception>
#include <string>

#ifndef ARM_CALIBRATION_FILE
#define ARM_CALIBRATION_FILE "servos.txt"
#endif

namespace {
constexpr double kSendPeriod = 0.02;  // s between pose lines to the Arduino (50 Hz)
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s <serial port>\n  e.g. %s /dev/ttyACM0\n"
                             "  (for the simulation alone, run arm_viewer)\n", argv[0], argv[0]);
        return 1;
    }
    const std::string port = argv[1];

    try {
        const arm::servo_calibration servos = arm::load_calibration(ARM_CALIBRATION_FILE);
        arm::serial_link link(port);

        bool alive = true;
        double sinceSend = kSendPeriod;  // send on the first frame
        std::string arduinoError;
        arm::run_viewer([&](const arm::arm_angles& pose, double dt) -> arm::viewer_status {
            if (!alive) return {"arm: disconnected, restart arm to reconnect", false};
            try {
                sinceSend += dt;
                if (sinceSend >= kSendPeriod) {
                    sinceSend = 0.0;
                    link.write_line(arm::to_arduino_line(arm::to_servo_commands(pose, servos)));
                }
                for (const std::string& l : link.poll_lines()) arduinoError = l;
            } catch (const std::exception& e) {
                std::fprintf(stderr, "%s\n", e.what());
                alive = false;
            }
            if (!arduinoError.empty()) return {"arduino: " + arduinoError, false};
            return {"arm: " + port, true};
        });

        if (alive) link.write_line("R");  // window closed: let every servo go limp
    } catch (const std::exception& e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
}
