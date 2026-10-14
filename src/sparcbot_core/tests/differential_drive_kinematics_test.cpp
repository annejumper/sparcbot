#include <iomanip>
#include <iostream>

#include "sparcbot/kinematics/differential_drive_kinematics.hpp"

void printTest(
    const DifferentialDriveKinematics& kinematics,
    double linear_m_s,
    double angular_rad_s)
{
    const WheelVelocities wheels =
        kinematics.robotToWheelVelocities(linear_m_s, angular_rad_s);

    const RobotVelocity robot =
        kinematics.wheelToRobotVelocity(wheels.left_m_s, wheels.right_m_s);

    std::cout
        << std::fixed << std::setprecision(4)
        << "Input:  v=" << linear_m_s
        << " m/s, omega=" << angular_rad_s << " rad/s\n"
        << "Wheels: vL=" << wheels.left_m_s
        << " m/s, vR=" << wheels.right_m_s << " m/s\n"
        << "Back:   v=" << robot.linear_m_s
        << " m/s, omega=" << robot.angular_rad_s << " rad/s\n\n";
}

int main()
{
    DifferentialDriveKinematicsConfig config;

    // Replace this with your measured center-to-center wheel spacing.
    // config.base_width_m = 0.175; //actual
    config.base_width_m = 0.16;  //testing

    DifferentialDriveKinematics kinematics(config);

    std::cout << "Differential Drive Kinematics Test\n";
    std::cout << "==================================\n\n";

    // 1. Stopped
    printTest(kinematics, 0.0, 0.0);

    // 2. Straight forward
    printTest(kinematics, 0.10, 0.0);

    // 3. Straight backward
    printTest(kinematics, -0.10, 0.0);

    // 4. Rotate left in place
    printTest(kinematics, 0.0, 1.0);

    // 5. Rotate right in place
    printTest(kinematics, 0.0, -1.0);

    // 6. Forward + left turn
    printTest(kinematics, 0.10, 0.5);

    // 7. Forward + right turn
    printTest(kinematics, 0.10, -0.5);

    return 0;
}