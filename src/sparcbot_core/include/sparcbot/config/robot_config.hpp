#pragma once

// Robot configuration for the standalone C++ tests and tools.
//
// There are NO values in this file. Every function below reads the single
// parameter file shared with the ROS 2 nodes:
//
//     sparcbot_bringup/config/sparcbot.yaml
//
// To use a different file, set the environment variable SPARCBOT_CONFIG:
//
//     SPARCBOT_CONFIG=/path/to/other.yaml ./dual_motor_test
//
// If a value is missing from the file, the function throws and names the key.

#include <string>

#include "sparcbot/control/wheel_feedforward.hpp"
#include "sparcbot/estimation/wheel_state_estimator.hpp"
#include "sparcbot/hardware/motor.hpp"
#include "sparcbot/kinematics/differential_drive_kinematics.hpp"
#include "sparcbot/kinematics/wheel_geometry.hpp"

namespace sparcbot::config {

// Path of the parameter file in use.
std::string configPath();

// sparcbot_driver
MotorConfig leftMotor();
MotorConfig rightMotor();
WheelGeometryConfig wheelGeometry();
WheelStateEstimatorConfig wheelStateEstimator();

// diff_drive
DifferentialDriveKinematicsConfig differentialDriveKinematics();

// wheel_controller
WheelFeedforwardConfig leftWheelFeedforward();
WheelFeedforwardConfig rightWheelFeedforward();

} // namespace sparcbot::config
