#pragma once
#include "arm/kinematics.hpp"
#include <functional>
#include <string>

namespace arm {
    // A line for the simulation's panel; shown green when ok, red otherwise.
    struct simulation_status
    {
        std::string text;
        bool ok = true;
    };

    // Called once per frame with the pose the arm should be at right now (already moving at the
    // simulation's joint speed) and the frame time in seconds. Returns a status line for the panel.
    using pose_sink = std::function<simulation_status(const arm_angles& pose, double dt_s)>;

    // Opens the simulation window and runs until it is closed.
    // With no sink it is a simulation only; give it one to drive something real.
    void run_simulation(const pose_sink& on_pose = {});
}
