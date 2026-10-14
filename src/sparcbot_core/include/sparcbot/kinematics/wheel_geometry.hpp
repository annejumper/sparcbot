#pragma once

#include <stdexcept>

struct WheelGeometryConfig {
    double radius_m = 0.010;
};

class WheelGeometry {
public:
    explicit WheelGeometry(
        const WheelGeometryConfig& config)
        : radius_m_(config.radius_m)
    {
        if (radius_m_ <= 0.0) {
            throw std::invalid_argument(
                "Wheel radius must be > 0");
        }
    }

    double angularToLinear(
        double omega_rad_s) const
    {
        return radius_m_ * omega_rad_s;
    }

    double linearToAngular(
        double velocity_m_s) const
    {
        return velocity_m_s / radius_m_;
    }

    double radius() const
    {
        return radius_m_;
    }

private:
    double radius_m_;
};