#include "sparcbot/estimation/wheel_state_estimator.hpp"

#include <cmath>
#include <stdexcept>

WheelStateEstimator::WheelStateEstimator(
    const WheelStateEstimatorConfig& config)
    : config_(config)
{
    if (config_.counts_per_rev <= 0.0) {
        throw std::invalid_argument("counts_per_rev must be > 0");
    }
}

void WheelStateEstimator::reset(int64_t counts)
{
    previous_counts_ = counts;

    state_.counts = counts;
    state_.position_rad =
        static_cast<double>(counts) * TWO_PI / config_.counts_per_rev;
    state_.velocity_raw_rad_s = 0.0;
    state_.velocity_rad_s = 0.0;

    initialized_ = true;
}

WheelState WheelStateEstimator::update(int64_t counts, double dt_s)
{
    if (dt_s <= 0.0) {
        throw std::invalid_argument("dt_s must be > 0");
    }

    if (!initialized_) {
        reset(counts);
        return state_;
    }

    const int64_t delta_counts = counts - previous_counts_;
    previous_counts_ = counts;

    state_.counts = counts;

    // Wheel angle:
    //
    // theta = counts * 2*pi / counts_per_rev
    //
    state_.position_rad =
        static_cast<double>(counts) *
        TWO_PI / config_.counts_per_rev;

    // Raw finite-difference angular velocity:
    //
    // omega = delta_theta / delta_t
    //       = delta_counts * (2*pi / CPR) / dt
    //
    state_.velocity_raw_rad_s =
        static_cast<double>(delta_counts) *
        (TWO_PI / config_.counts_per_rev) /
        dt_s;

    if (config_.velocity_cutoff_hz <= 0.0) {
        state_.velocity_rad_s = state_.velocity_raw_rad_s;
        return state_;
    }

    // First-order low-pass filter:
    //
    // tau   = 1 / (2*pi*f_c)
    // alpha = dt / (tau + dt)
    //
    // omega_f[k] =
    //     omega_f[k-1] + alpha * (omega_raw[k] - omega_f[k-1])
    //
    const double tau =
        1.0 / (TWO_PI * config_.velocity_cutoff_hz);

    const double alpha =
        dt_s / (tau + dt_s);

    state_.velocity_rad_s +=
        alpha *
        (state_.velocity_raw_rad_s - state_.velocity_rad_s);

    return state_;
}
