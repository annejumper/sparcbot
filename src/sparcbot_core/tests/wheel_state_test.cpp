#include "sparcbot/config/robot_config.hpp"
#include "sparcbot/hardware/motor.hpp"
#include "sparcbot/hardware/robot_motors.hpp"
#include "sparcbot/estimation/wheel_state_estimator.hpp"

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t keep_running = 1;

void signalHandler(int)
{
    keep_running = 0;
}

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const MotorConfig left = sparcbot::config::leftMotor();
    const MotorConfig right = sparcbot::config::rightMotor();

    try {
        RobotMotors motors(left, right);

        // Use the same estimator implementation for both wheels.
        WheelStateEstimator left_estimator(sparcbot::config::wheelStateEstimator());
        WheelStateEstimator right_estimator(sparcbot::config::wheelStateEstimator());

        motors.resetEncoders();
        left_estimator.reset(0);
        right_estimator.reset(0);

        // Start with a conservative open-loop command.
        constexpr double TEST_COMMAND = 0.20;

        // Run the state estimator at 100 Hz for this test.
        // Later the controller loop can use its own measured dt.
        constexpr double TARGET_DT = 0.01;

        std::cout
            << "\nWheel State Estimator Test\n"
            << "==========================\n"
            << "Both wheels will run forward at u = "
            << TEST_COMMAND << "\n"
            << "Ctrl-C stops both motors.\n\n";

        motors.setMotorCommands(TEST_COMMAND, TEST_COMMAND);

        auto previous_time = std::chrono::steady_clock::now();

        while (keep_running) {
            const auto loop_start = std::chrono::steady_clock::now();

            const double dt =
                std::chrono::duration<double>(
                    loop_start - previous_time).count();

            previous_time = loop_start;

            const auto motor_state = motors.getState();

            const WheelState left_state =
                left_estimator.update(motor_state.left.counts, dt);

            const WheelState right_state =
                right_estimator.update(motor_state.right.counts, dt);

            std::cout   
                << std::fixed << std::setprecision(3)

                << "L: count=" << std::setw(7) << left_state.counts
                << " theta=" << std::setw(8) << left_state.position_rad
                << " raw_w=" << std::setw(8) << left_state.velocity_raw_rad_s
                << " filt_w=" << std::setw(8) << left_state.velocity_rad_s

                << " | "

                << "R: count=" << std::setw(7) << right_state.counts
                << " theta=" << std::setw(8) << right_state.position_rad
                << " raw_w=" << std::setw(8) << right_state.velocity_raw_rad_s
                << " filt_w=" << std::setw(8) << right_state.velocity_rad_s

                << "      "
                << '\n';

            const auto elapsed =
                std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - loop_start).count();

            const double sleep_time = TARGET_DT - elapsed;

            if (sleep_time > 0.0) {
                std::this_thread::sleep_for(
                    std::chrono::duration<double>(sleep_time));
            }
        }

        motors.coast();
        std::cout << "\n\nStopped. Both motors are coasting.\n";
    }
    catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << '\n';
        return 1;
    }

    return 0;
}
