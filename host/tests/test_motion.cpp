#include "arm/motion.hpp"
#include <gtest/gtest.h>

using namespace arm;
constexpr double kTol = 1e-9;

TEST(StepToward, MovesAtMostTheSpeedLimit) {
    arm_angles a = step_toward({}, {.shoulder = 1.0}, 2.0, 0.1);
    EXPECT_NEAR(a.shoulder, 0.2, kTol);
}

TEST(StepToward, MovesBackwardsToo) {
    arm_angles a = step_toward({}, {.elbow = -1.0}, 2.0, 0.1);
    EXPECT_NEAR(a.elbow, -0.2, kTol);
}

TEST(StepToward, LandsExactlyOnTargetWithoutOvershoot) {
    arm_angles a = step_toward({}, {.base = 0.05}, 2.0, 0.1);
    EXPECT_NEAR(a.base, 0.05, kTol);
}

TEST(StepToward, EachJointMovesIndependently) {
    arm_angles a = step_toward({}, {.base = 1.0, .wrist = 0.01, .gripper = -1.0}, 1.0, 0.1);
    EXPECT_NEAR(a.base, 0.1, kTol);
    EXPECT_NEAR(a.wrist, 0.01, kTol);
    EXPECT_NEAR(a.gripper, -0.1, kTol);
    EXPECT_NEAR(a.shoulder, 0.0, kTol);
}

TEST(StepToward, RepeatedStepsArrive) {
    arm_angles a{};
    const arm_angles target{.base = 1, .shoulder = -0.5, .elbow = 2};
    for (int i = 0; i < 180; ++i) a = step_toward(a, target, 1.0, 1.0 / 60);  // 3 s covers the 2 rad elbow move
    EXPECT_NEAR(a.base, 1, kTol);
    EXPECT_NEAR(a.shoulder, -0.5, kTol);
    EXPECT_NEAR(a.elbow, 2, kTol);
}
