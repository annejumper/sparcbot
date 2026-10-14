#pragma once

// Feedforward parameters for one wheel (wheel_controller.<wheel>.feedforward in
// sparcbot_bringup/config/sparcbot.yaml). The feedforward itself is computed in
// Python: sparcbot_control/sparcbot_control/feedforward.py.
struct WheelFeedforwardConfig {
    double forward_static = 0.0;
    double forward_velocity_gain = 0.0;

    double reverse_static = 0.0;
    double reverse_velocity_gain = 0.0;

    double forward_breakaway = 0.0;
    double reverse_breakaway = 0.0;

    double stopped_velocity_threshold = 0.005;

    double max_command = 1.0;
    double zero_velocity_threshold = 1e-4;
};
