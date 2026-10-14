#pragma once
#include "sparcbot/hardware/motor.hpp"

struct RobotMotorState {
    MotorState left;
    MotorState right;
};

class RobotMotors {
public:
    RobotMotors(const MotorConfig& left_cfg, const MotorConfig& right_cfg)
        : left_(left_cfg), right_(right_cfg) {}

    void setMotorCommands(double u_left, double u_right) {
        left_.setCommand(u_left);
        right_.setCommand(u_right);
    }

    RobotMotorState getState() const {
        return {left_.getState(), right_.getState()};
    }

    void coast() {
        left_.coast();
        right_.coast();
    }

    void brake() {
        left_.brake();
        right_.brake();
    }

    void resetEncoders() {
        left_.resetEncoder();
        right_.resetEncoder();
    }

    Motor& left() { return left_; }
    Motor& right() { return right_; }

    const Motor& left() const { return left_; }
    const Motor& right() const { return right_; }

private:
    Motor left_;
    Motor right_;
};