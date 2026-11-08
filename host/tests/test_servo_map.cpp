#include "arm/servo_map.hpp"
#include <cstdio>
#include <fstream>
#include <gtest/gtest.h>
#include <numbers>

using namespace arm;
constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-9;

// every joint calibrated with the defaults: command 90 = 0 deg, command 180 = 90 deg
servo_calibration allDefault() { return {}; }

TEST(ServoMap, ZeroAngleIsTheZeroCommand) {
    servo_calibration cal = allDefault();
    cal[1].cmd_a = 40;
    cal[1].cmd_b = 130;
    auto d = to_servo_commands({}, cal);
    EXPECT_NEAR(d[0], 90, kTol);
    EXPECT_NEAR(d[1], 40, kTol);
}

TEST(ServoMap, ConvertsRadiansToDegrees) {
    auto d = to_servo_commands({.base = kPi / 4}, allDefault());
    EXPECT_NEAR(d[0], 135, kTol);
}

TEST(ServoMap, MirroredServoGoesTheOtherWay) {
    servo_calibration cal = allDefault();
    cal[2].cmd_b = 0;
    auto d = to_servo_commands({.elbow = kPi / 4}, cal);
    EXPECT_NEAR(d[2], 45, kTol);
}

TEST(ServoMap, ShortTravelServoIsScaled) {
    // a servo that only turns 80 deg for commands 90..180
    servo_calibration cal = allDefault();
    cal[1].joint_b = 80;
    auto d = to_servo_commands({.shoulder = kPi * 40 / 180}, cal);
    EXPECT_NEAR(d[1], 135, kTol);
}

TEST(ServoMap, ClampsToLimits) {
    servo_calibration cal = allDefault();
    cal[3].max_cmd = 150;
    auto d = to_servo_commands({.wrist = kPi / 2, .gripper = -kPi}, cal);
    EXPECT_NEAR(d[3], 150, kTol);
    EXPECT_NEAR(d[5], 0, kTol);
}

TEST(ServoMap, JointOrderMatchesFirmware) {
    auto d = to_servo_commands({.base = 0.1, .shoulder = 0.2, .elbow = 0.3, .wrist = 0.4, .wrist_spin = 0.5, .gripper = 0.6},
                              allDefault());
    for (int i = 0; i < 6; ++i) EXPECT_NEAR(d[i], 90 + (i + 1) * 0.1 * 180 / kPi, kTol);
}

TEST(ArduinoLine, FormatsSixCommands) {
    EXPECT_EQ(to_arduino_line({90, 45.26, 0, 180, 12.34, 7}), "P 90.0 45.3 0.0 180.0 12.3 7.0");
}

TEST(CalibrationFile, MissingFileMeansNothingCalibratedYet) {
    partial_calibration cal = load_partial_calibration("/nonexistent/servos.txt");
    for (const auto& c : cal) EXPECT_FALSE(c.has_value());
}

TEST(CalibrationFile, ArmRefusesAnIncompleteCalibration) {
    const std::string path = testing::TempDir() + "servos_partial.txt";
    std::ofstream(path) << "shoulder 0 90 90 180 0 180\n";
    EXPECT_THROW(load_calibration(path), std::runtime_error);
    EXPECT_TRUE(load_partial_calibration(path)[1].has_value());
    std::remove(path.c_str());
}

TEST(CalibrationFile, SaveThenLoadRoundTrips) {
    const std::string path = testing::TempDir() + "servos_roundtrip.txt";
    partial_calibration cal{};
    cal[1] = servo_cal{.joint_a = 0, .cmd_a = 97, .joint_b = 90, .cmd_b = 3, .min_cmd = 15, .max_cmd = 172};
    cal[5] = servo_cal{.joint_a = 0, .cmd_a = 20, .joint_b = 45, .cmd_b = 80, .min_cmd = 10, .max_cmd = 100};
    save_calibration(path, cal);
    partial_calibration back = load_partial_calibration(path);
    std::remove(path.c_str());

    EXPECT_FALSE(back[0].has_value());
    ASSERT_TRUE(back[1].has_value());
    EXPECT_NEAR(back[1]->cmd_a, 97, kTol);
    EXPECT_NEAR(back[1]->cmd_b, 3, kTol);
    EXPECT_NEAR(back[1]->max_cmd, 172, kTol);
    ASSERT_TRUE(back[5].has_value());
    EXPECT_NEAR(back[5]->joint_b, 45, kTol);
}

TEST(CalibrationFile, RejectsUnknownJoint) {
    const std::string path = testing::TempDir() + "servos_bad.txt";
    std::ofstream(path) << "knee 0 90 90 180 0 180\n";
    EXPECT_THROW(load_partial_calibration(path), std::runtime_error);
    std::remove(path.c_str());
}

TEST(ServoTravel, CoversTheCommandRange) {
    // default: command 90 = 0 deg, 180 = 90 deg, so 0..180 reaches -90..90 deg
    const joint_range r = servo_travel(servo_cal{});
    EXPECT_NEAR(r.min, -kPi / 2, kTol);
    EXPECT_NEAR(r.max, kPi / 2, kTol);
}

TEST(ServoTravel, MirroredServoStillGivesMinBelowMax) {
    // like the wrist roll: 0 deg at command 180, 90 deg at command 90
    const joint_range r = servo_travel({.joint_a = 0, .cmd_a = 180, .joint_b = 90, .cmd_b = 90});
    EXPECT_NEAR(r.min, 0, kTol);
    EXPECT_NEAR(r.max, kPi, kTol);
}

TEST(ServoTravel, NarrowsLimitsOnlyWhereTheServoFallsShort) {
    servo_calibration cal = allDefault();  // every joint reaches -90..90 deg
    const arm_limits wide{.base = {-1, 1}, .shoulder = {-2, 2}, .elbow = {-1, 3}, .wrist = {-3, 1},
                          .wrist_spin = {-0.5, 0.5}, .gripper = {0, 1}, .ground_clearance = 7};
    const arm_limits l = within_servo_travel(wide, cal);
    EXPECT_NEAR(l.base.min, -1, kTol);
    EXPECT_NEAR(l.shoulder.min, -kPi / 2, kTol);
    EXPECT_NEAR(l.shoulder.max, kPi / 2, kTol);
    EXPECT_NEAR(l.elbow.max, kPi / 2, kTol);
    EXPECT_NEAR(l.wrist.min, -kPi / 2, kTol);
    EXPECT_NEAR(l.wrist_spin.max, 0.5, kTol);
    EXPECT_EQ(l.ground_clearance, 7);
}
