#pragma once
#include "arm/kinematics.hpp"
#include <array>
#include <optional>
#include <string>

namespace arm {
    // Joint names in firmware order, as used in the calibration file.
    inline constexpr std::array<const char*, 6> kJointNames{"base", "shoulder", "elbow", "wrist", "wrist_spin", "gripper"};

    // How one servo maps onto its joint, measured on the real arm at two poses (see arm_calibrate).
    // "Command" is the number sent to the servo; "joint" is the angle the kinematics use (degrees here, for easy measuring).
    // Two points cover the offset, the direction (mirrored servos) and the scale (a servo that only
    // travels ~160 deg for commands 0..180). Commands are clamped to [min_cmd, max_cmd].
    struct servo_cal
    {
        double joint_a = 0;       // at this joint angle...
        double cmd_a = 90;        // ...the servo needs this command
        double joint_b = 90;      // a second pose, as far from the first as is convenient
        double cmd_b = 180;
        double min_cmd = 0;       // where the joint hits something (or the servo stops moving)
        double max_cmd = 180;
    };

    // Every servo's calibration, firmware order: base, shoulder, elbow, wrist pitch, wrist roll, gripper.
    using servo_calibration = std::array<servo_cal, 6>;

    // The command each servo needs for the arm to reach `pose`, firmware order.
    std::array<double, 6> to_servo_commands(const arm_angles& pose, const servo_calibration& cal);

    // The line of text the Arduino reads to move all six servos, e.g. "P 90.0 45.0 120.0 90.0 90.0 30.0".
    // No trailing newline; serial_link::write_line adds it.
    std::string to_arduino_line(const std::array<double, 6>& servo_commands);

    // Calibration file: one line per calibrated joint, '#' starts a comment.
    //   <name> <joint_a> <cmd_a> <joint_b> <cmd_b> <min_cmd> <max_cmd>
    // Throws std::runtime_error on a malformed line, or if any joint is missing ("run arm_calibrate").
    servo_calibration load_calibration(const std::string& path);

    // The file as it is part way through calibrating: joints not done yet are empty.
    // Only arm_calibrate needs this. A missing file means nothing is calibrated yet.
    using partial_calibration = std::array<std::optional<servo_cal>, 6>;
    partial_calibration load_partial_calibration(const std::string& path);
    void save_calibration(const std::string& path, const partial_calibration& cal);
}
