#pragma once

#include <stdexcept>

struct RobotVelocity {
    double linear_m_s = 0.0;
    double angular_rad_s = 0.0;
};

struct WheelVelocities {
    double left_m_s = 0.0;
    double right_m_s = 0.0;
};

struct DifferentialDriveKinematicsConfig {
    double base_width_m = 0.0;
};

class DifferentialDriveKinematics {
public:
    explicit DifferentialDriveKinematics(const DifferentialDriveKinematicsConfig& config);

    WheelVelocities robotToWheelVelocities(double linear_m_s, double angular_rad_s) const;
    RobotVelocity wheelToRobotVelocity(double left_m_s, double right_m_s) const;

    double baseWidth() const;

private:
    double base_width_m_;
};