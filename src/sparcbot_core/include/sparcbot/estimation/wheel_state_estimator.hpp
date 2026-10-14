#pragma once

#include <cstdint>

struct WheelState {
    int64_t counts = 0;

    // Absolute wheel position from encoder zero.
    double position_rad = 0.0;

    // Direct finite-difference velocity from encoder counts.
    double velocity_raw_rad_s = 0.0;

    // Low-pass filtered wheel velocity.
    double velocity_rad_s = 0.0;
};

struct WheelStateEstimatorConfig {
    // Encoder counts per gearbox output revolution.
    double counts_per_rev = 500.0;

    // First-order low-pass cutoff frequency for velocity.
    // Set <= 0 to disable filtering.
    double velocity_cutoff_hz = 8.0;
};

class WheelStateEstimator {
public:
    explicit WheelStateEstimator(
        const WheelStateEstimatorConfig& config = WheelStateEstimatorConfig{});

    // Reset estimator history.
    // Call this when encoder zero is reset or before starting a test.
    void reset(int64_t counts = 0);

    // Update using the newest encoder count and the elapsed time since
    // the previous update.
    //
    // dt_s must be positive.
    WheelState update(int64_t counts, double dt_s);

    const WheelState& state() const { return state_; }

private:
    WheelStateEstimatorConfig config_;
    WheelState state_;

    int64_t previous_counts_ = 0;
    bool initialized_ = false;

    static constexpr double TWO_PI =
        6.283185307179586476925286766559;
};
