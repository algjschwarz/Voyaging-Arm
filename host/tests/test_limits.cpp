#include "arm/limits.hpp"
#include "arm/presets.hpp"
#include <gtest/gtest.h>
#include <numbers>

using namespace arm;
constexpr double kTol = 1e-9;
constexpr double deg(double d) { return d * std::numbers::pi / 180.0; }

// Fixed test limits, so these tests don't move when the real ones are measured again.
constexpr arm_limits kLimits{
    .base = {deg(-90), deg(90)},
    .shoulder = {deg(0), deg(170)},
    .elbow = {deg(-150), deg(150)},
    .wrist = {deg(-120), deg(120)},
    .ground_clearance = 10.0,
};

TEST(JointPoints, TipMatchesForward) {
    const arm_angles a{.base = 0.4, .shoulder = 1.1, .elbow = -0.7, .wrist = -0.3};
    const position tip = joint_points(a, VOYAGING_ARM)[5];
    const position f = forward(a, VOYAGING_ARM);
    EXPECT_NEAR(tip.x, f.x, kTol);
    EXPECT_NEAR(tip.y, f.y, kTol);
    EXPECT_NEAR(tip.z, f.z, kTol);
}

TEST(WhyUnsafe, StartingPoseIsSafe) {
    const arm_angles a = inverse({280, 0, 150}, VOYAGING_ARM, 0.0);
    EXPECT_EQ(why_unsafe(a, VOYAGING_ARM, kLimits), "");
    EXPECT_EQ(why_unsafe(a, VOYAGING_ARM, VOYAGING_ARM_LIMITS), "");
}

TEST(WhyUnsafe, NamesTheJointPastItsLimit) {
    const arm_angles safe = inverse({280, 0, 150}, VOYAGING_ARM, 0.0);
    arm_angles a = safe;
    a.base = deg(91);
    EXPECT_EQ(why_unsafe(a, VOYAGING_ARM, kLimits), "base past 90 deg");
    a = safe;
    a.shoulder = deg(-1);
    EXPECT_EQ(why_unsafe(a, VOYAGING_ARM, kLimits), "shoulder past 0 deg");
    a = safe;
    a.elbow = deg(-151);
    EXPECT_EQ(why_unsafe(a, VOYAGING_ARM, kLimits), "elbow past -150 deg");
    a = safe;
    a.wrist = deg(121);
    EXPECT_EQ(why_unsafe(a, VOYAGING_ARM, kLimits), "wrist past 120 deg");
}

TEST(WhyUnsafe, KeepsTheTipOffTheTable) {
    // claw pointing straight down
    const arm_angles low = inverse({200, 0, 5}, VOYAGING_ARM, deg(-90));
    EXPECT_EQ(why_unsafe(low, VOYAGING_ARM, kLimits), "tip 5 mm above the table");
    const arm_angles high = inverse({200, 0, 15}, VOYAGING_ARM, deg(-90));
    EXPECT_EQ(why_unsafe(high, VOYAGING_ARM, kLimits), "");
}
