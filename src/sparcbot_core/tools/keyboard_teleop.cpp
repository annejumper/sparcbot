#include "sparcbot/config/robot_config.hpp"
#include "sparcbot/hardware/robot_motors.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <fcntl.h>

static volatile std::sig_atomic_t keep_running = 1;

void signalHandler(int) {
    keep_running = 0;
}

class RawTerminal {
public:
    RawTerminal() {
        if (tcgetattr(STDIN_FILENO, &old_term_) < 0) {
            throw std::runtime_error("tcgetattr failed");
        }

        termios raw = old_term_;
        raw.c_lflag &= ~(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;

        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) < 0) {
            throw std::runtime_error("tcsetattr failed");
        }

        old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK);
    }

    ~RawTerminal() {
        tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }

private:
    termios old_term_{};
    int old_flags_ = 0;
};

bool readKey(char& c) {
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return n == 1;
}

int main() {
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const MotorConfig left = sparcbot::config::leftMotor();
    const MotorConfig right = sparcbot::config::rightMotor();

    try {
        RobotMotors motors(left, right);
        RawTerminal terminal;

        double linear_cmd = 0.20;
        double turn_cmd = 0.16;

        constexpr double MIN_CMD = 0.05;
        constexpr double MAX_CMD = 0.60;
        constexpr double STEP = 0.05;

        double left_u = 0.0;
        double right_u = 0.0;

        std::cout
            << "\nKeyboard Teleop\n"
            << "===============\n"
            << "  w : forward\n"
            << "  s : backward\n"
            << "  a : turn left in place\n"
            << "  d : turn right in place\n"
            << "  space : stop/coast\n"
            << "  b : brake\n"
            << "  + / = : increase drive command\n"
            << "  -     : decrease drive command\n"
            << "  ]     : increase turn command\n"
            << "  [     : decrease turn command\n"
            << "  r     : reset encoder counts\n"
            << "  q     : quit\n\n"
            << "Open-loop PWM teleop: equal commands may not give equal wheel speeds.\n\n";

        auto last_print = std::chrono::steady_clock::now();

        while (keep_running) {
            char key;

            if (readKey(key)) {
                switch (key) {
                    case 'w': case 'W':
                        left_u = +linear_cmd;
                        right_u = +linear_cmd;
                        motors.setMotorCommands(left_u, right_u);
                        break;

                    case 's': case 'S':
                        left_u = -linear_cmd;
                        right_u = -linear_cmd;
                        motors.setMotorCommands(left_u, right_u);
                        break;

                    case 'a': case 'A':
                        left_u = -turn_cmd;
                        right_u = +turn_cmd;
                        motors.setMotorCommands(left_u, right_u);
                        break;

                    case 'd': case 'D':
                        left_u = +turn_cmd;
                        right_u = -turn_cmd;
                        motors.setMotorCommands(left_u, right_u);
                        break;

                    case ' ':
                        left_u = 0.0;
                        right_u = 0.0;
                        motors.coast();
                        break;

                    case 'b': case 'B':
                        left_u = 0.0;
                        right_u = 0.0;
                        motors.brake();
                        break;

                    case '+': case '=':
                        linear_cmd = std::min(MAX_CMD, linear_cmd + STEP);
                        std::cout << "\nDrive command = " << std::fixed << std::setprecision(2)
                                  << linear_cmd << '\n';
                        break;

                    case '-': case '_':
                        linear_cmd = std::max(MIN_CMD, linear_cmd - STEP);
                        std::cout << "\nDrive command = " << std::fixed << std::setprecision(2)
                                  << linear_cmd << '\n';
                        break;

                    case ']':
                        turn_cmd = std::min(MAX_CMD, turn_cmd + STEP);
                        std::cout << "\nTurn command = " << std::fixed << std::setprecision(2)
                                  << turn_cmd << '\n';
                        break;

                    case '[':
                        turn_cmd = std::max(MIN_CMD, turn_cmd - STEP);
                        std::cout << "\nTurn command = " << std::fixed << std::setprecision(2)
                                  << turn_cmd << '\n';
                        break;

                    case 'r': case 'R':
                        motors.resetEncoders();
                        std::cout << "\nEncoders reset.\n";
                        break;

                    case 'q': case 'Q':
                        keep_running = 0;
                        break;

                    default:
                        break;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - last_print).count() >= 0.2) {
                const auto state = motors.getState();

                std::cout
                    << "\r"
                    << std::fixed << std::setprecision(2)
                    << "cmd L=" << std::setw(5) << left_u
                    << " R=" << std::setw(5) << right_u
                    << " | enc L=" << std::setw(8) << state.left.counts
                    << " R=" << std::setw(8) << state.right.counts
                    << " | drive=" << linear_cmd
                    << " turn=" << turn_cmd
                    << "      "
                    << std::flush;

                last_print = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        motors.coast();
        std::cout << "\n\nStopped. Both motors are coasting.\n";

    } catch (const std::exception& e) {
        std::cerr << "\nERROR: " << e.what() << '\n';
        return 1;
    }

    return 0;
}