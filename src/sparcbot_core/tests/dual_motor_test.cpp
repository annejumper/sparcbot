#include "sparcbot/config/robot_config.hpp"
#include "sparcbot/hardware/robot_motors.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

static volatile std::sig_atomic_t keep_running = 1;
void signalHandler(int) { keep_running = 0; }

void printState(const RobotMotors& motors) {
    auto s = motors.getState();
    std::cout << std::fixed << std::setprecision(3)
              << "LEFT u=" << std::setw(6) << s.left.command
              << " count=" << std::setw(8) << s.left.counts
              << " pos=" << std::setw(8) << s.left.position_rad << " rad"
              << " | RIGHT u=" << std::setw(6) << s.right.command
              << " count=" << std::setw(8) << s.right.counts
              << " pos=" << std::setw(8) << s.right.position_rad << " rad\n";
}

void runFor(RobotMotors& motors, double left_u, double right_u, double seconds) {
    motors.setMotorCommands(left_u, right_u);
    auto start = std::chrono::steady_clock::now();
    while (keep_running) {
        printState(motors);
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count() >= seconds) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    motors.coast();
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const MotorConfig left = sparcbot::config::leftMotor();
    const MotorConfig right = sparcbot::config::rightMotor();

    try {
        RobotMotors motors(left, right);
        std::cout << "\nLift the robot so both wheels can spin freely. Ctrl-C stops.\n";
        constexpr double U = 0.20;
        constexpr double T = 1.5;

        std::cout << "\n[1] Left +\n";  runFor(motors, +U, 0.0, T); if (!keep_running) return 0;
        std::cout << "\n[2] Left -\n";  runFor(motors, -U, 0.0, T); if (!keep_running) return 0;
        std::cout << "\n[3] Right +\n"; runFor(motors, 0.0, +U, T); if (!keep_running) return 0;
        std::cout << "\n[4] Right -\n"; runFor(motors, 0.0, -U, T); if (!keep_running) return 0;
        std::cout << "\n[5] Both +\n";  runFor(motors, +U, +U, T);

        motors.coast();
        std::cout << "\nDone.\n";
    } catch (const std::exception& e) {
        std::cerr << "ERROR: " << e.what() << '\n';
        return 1;
    }
    return 0;
}
