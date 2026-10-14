#include "sparcbot/kinematics/differential_drive_kinematics.hpp"

DifferentialDriveKinematics::DifferentialDriveKinematics(const DifferentialDriveKinematicsConfig& config)
    : base_width_m_(config.base_width_m)
{
    if (base_width_m_ <= 0.0) {
        throw std::invalid_argument("Track width must be > 0");
    }
}

WheelVelocities DifferentialDriveKinematics::robotToWheelVelocities(double linear_m_s, double angular_rad_s) const
{
    WheelVelocities velocities;

    velocities.left_m_s = linear_m_s - 0.5 * base_width_m_ * angular_rad_s;
    velocities.right_m_s = linear_m_s + 0.5 * base_width_m_ * angular_rad_s;

    return velocities;
}

RobotVelocity DifferentialDriveKinematics::wheelToRobotVelocity(double left_m_s, double right_m_s) const
{
    RobotVelocity velocity;

    velocity.linear_m_s = 0.5 * (left_m_s + right_m_s);
    velocity.angular_rad_s = (right_m_s - left_m_s) / base_width_m_;

    return velocity;
}

double DifferentialDriveKinematics::baseWidth() const
{
    return base_width_m_;
}