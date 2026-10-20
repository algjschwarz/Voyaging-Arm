#include "arm/kinematics.hpp"
#include "arm/presets.hpp"
#include <gtest/gtest.h>
#include <numbers>

using namespace arm;
constexpr double kPi = std::numbers::pi;
constexpr double kTol = 1e-9;

// A textbook arm: unit links, no offsets, shoulder on the ground.
constexpr arm_constants kTextbook{
    .shoulder_height = 0, .length1 = 1, .length2 = 1, .length3 = 1,
    .shoulder_offset = 0, .wrist_offset = 0};

TEST(Forward, TextbookStraightOutReachesSumOfLinks) {
    position p = forward({}, kTextbook);
    EXPECT_NEAR(p.x, 3, kTol);
    EXPECT_NEAR(p.y, 0, kTol);
    EXPECT_NEAR(p.z, 0, kTol);
}

TEST(Forward, TextbookStraightUp) {
    position p = forward({.shoulder = kPi / 2}, kTextbook);
    EXPECT_NEAR(p.x, 0, kTol);
    EXPECT_NEAR(p.z, 3, kTol);
}

TEST(Forward, BaseYawSwingsTheTipIntoY) {
    position p = forward({.base = kPi / 2}, kTextbook);
    EXPECT_NEAR(p.x, 0, kTol);
    EXPECT_NEAR(p.y, 3, kTol);
}

TEST(Forward, RealArmAtZeroShowsTheOffsets) {
    const arm_constants& c = VOYAGING_ARM;
    position p = forward({}, c);
    EXPECT_NEAR(p.x, c.shoulder_offset + c.length1 + c.length2 + c.length3, kTol);
    EXPECT_NEAR(p.z, c.shoulder_height + c.wrist_offset, kTol);
}

TEST(Inverse, RoundTripsThroughForwardOnTheRealArm) {
    const arm_constants& c = VOYAGING_ARM;
    const arm_angles cases[] = {
        {.base = 0.3, .shoulder = 1.0, .elbow = 0.8, .wrist = -0.5},
        {.base = -1.2, .shoulder = 0.4, .elbow = 1.2, .wrist = 0.2},
        {.base = 2.0, .shoulder = 1.3, .elbow = 0.3, .wrist = -1.0},
    };
    for (const arm_angles& in : cases) {
        position p = forward(in, c);
        arm_angles out = inverse(p, c, in.shoulder + in.elbow + in.wrist);
        EXPECT_NEAR(out.base, in.base, kTol);
        EXPECT_NEAR(out.shoulder, in.shoulder, kTol);
        EXPECT_NEAR(out.elbow, in.elbow, kTol);
        EXPECT_NEAR(out.wrist, in.wrist, kTol);
        position back = forward(out, c);
        EXPECT_NEAR(back.x, p.x, 1e-6);
        EXPECT_NEAR(back.y, p.y, 1e-6);
        EXPECT_NEAR(back.z, p.z, 1e-6);
    }
}
