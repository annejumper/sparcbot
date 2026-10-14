#include "sparcbot/config/robot_config.hpp"
#include "sparcbot/hardware/motor.hpp"
#include "sparcbot/hardware/robot_motors.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

// ============================================================
// GLOBAL
// ============================================================

static volatile std::sig_atomic_t keep_running = 1;

void signalHandler(int)
{
    keep_running = 0;
}

// ============================================================
// TEST CONFIGURATION
// ============================================================

constexpr int REST_MS = 700;
constexpr int SAMPLE_MS = 5;
constexpr int MAX_PULSE_MS = 150;

constexpr long long MOTION_THRESHOLD_COUNTS = 2;

// A candidate is considered reliable only if it passes
// every one of these trials.
constexpr int NUM_TRIALS = 5;

// Automatic search step.
constexpr double SEARCH_STEP = 0.01;

// Maximum command magnitude allowed during automatic search.
constexpr double MAX_SEARCH_COMMAND = 0.25;

// Starting candidates.
constexpr double FORWARD_LEFT_START = 0.15;
constexpr double FORWARD_RIGHT_START = 0.10;

constexpr double REVERSE_LEFT_START = 0.13;
constexpr double REVERSE_RIGHT_START = 0.13;

// ============================================================
// RESULT TYPES
// ============================================================

struct WheelStartupResult {
    bool moved = false;
    long long delta_counts = 0;
    double first_motion_time_s = -1.0;
};

struct StartupTrialResult {
    WheelStartupResult left;
    WheelStartupResult right;

    bool both_moved = false;
    double start_time_difference_s = -1.0;
};

struct CandidateResult {
    int successful_trials = 0;
    bool reliable = false;

    double avg_left_start_time_s = 0.0;
    double avg_right_start_time_s = 0.0;
    double avg_start_difference_s = 0.0;
};

// ============================================================
// UTILITIES
// ============================================================

bool sleepInterruptible(int ms)
{
    while (keep_running && ms > 0) {
        const int chunk = std::min(ms, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        ms -= chunk;
    }

    return keep_running;
}

void coastAll(RobotMotors& motors)
{
    motors.coast();
}

// ============================================================
// RUN ONE STARTUP TRIAL
// ============================================================

StartupTrialResult runStartupTrial(RobotMotors& motors, double u_left, double u_right)
{
    StartupTrialResult result;

    coastAll(motors);

    if (!sleepInterruptible(REST_MS)) {
        return result;
    }

    const RobotMotorState start_state = motors.getState();

    const long long left_start = start_state.left.counts;
    const long long right_start = start_state.right.counts;

    const int left_direction = (u_left >= 0.0) ? +1 : -1;
    const int right_direction = (u_right >= 0.0) ? +1 : -1;

    motors.setMotorCommands(u_left, u_right);

    const auto start_time = std::chrono::steady_clock::now();

    bool left_detected = false;
    bool right_detected = false;

    for (int elapsed_ms = 0;
         elapsed_ms < MAX_PULSE_MS && keep_running;
         elapsed_ms += SAMPLE_MS) {

        sleepInterruptible(SAMPLE_MS);

        const auto now = std::chrono::steady_clock::now();
        const RobotMotorState state = motors.getState();

        const long long left_delta = state.left.counts - left_start;
        const long long right_delta = state.right.counts - right_start;

        const long long left_progress = left_direction * left_delta;
        const long long right_progress = right_direction * right_delta;

        if (!left_detected && left_progress >= MOTION_THRESHOLD_COUNTS) {
            left_detected = true;
            result.left.moved = true;
            result.left.first_motion_time_s =
                std::chrono::duration<double>(now - start_time).count();
        }

        if (!right_detected && right_progress >= MOTION_THRESHOLD_COUNTS) {
            right_detected = true;
            result.right.moved = true;
            result.right.first_motion_time_s =
                std::chrono::duration<double>(now - start_time).count();
        }

        // Stop immediately once both wheels have moved enough.
        if (left_detected && right_detected) {
            break;
        }
    }

    coastAll(motors);

    const RobotMotorState end_state = motors.getState();

    result.left.delta_counts = end_state.left.counts - left_start;
    result.right.delta_counts = end_state.right.counts - right_start;

    result.both_moved = result.left.moved && result.right.moved;

    if (result.both_moved) {
        result.start_time_difference_s =
            std::abs(result.left.first_motion_time_s -
                     result.right.first_motion_time_s);
    }

    return result;
}

// ============================================================
// PRINT ONE TRIAL
// ============================================================

void printTrialResult(int trial_number, const StartupTrialResult& result)
{
    std::cout << "  Trial " << trial_number << ": "
              << (result.both_moved ? "PASS" : "FAIL")
              << "   L dN=" << result.left.delta_counts
              << "   R dN=" << result.right.delta_counts;

    if (result.left.moved) {
        std::cout << "   tL="
                  << std::fixed
                  << std::setprecision(3)
                  << result.left.first_motion_time_s
                  << " s";
    }
    else {
        std::cout << "   tL=NO";
    }

    if (result.right.moved) {
        std::cout << "   tR="
                  << std::fixed
                  << std::setprecision(3)
                  << result.right.first_motion_time_s
                  << " s";
    }
    else {
        std::cout << "   tR=NO";
    }

    if (result.both_moved) {
        std::cout << "   |dt|="
                  << std::fixed
                  << std::setprecision(3)
                  << result.start_time_difference_s
                  << " s";
    }

    std::cout << '\n';
}

// ============================================================
// TEST ONE COMMAND PAIR
// ============================================================

CandidateResult testCandidate(RobotMotors& motors,
                              double u_left,
                              double u_right,
                              bool stop_on_first_failure)
{
    CandidateResult result;

    double total_left_time = 0.0;
    double total_right_time = 0.0;
    double total_difference = 0.0;

    for (int trial = 1; trial <= NUM_TRIALS && keep_running; ++trial) {
        const StartupTrialResult trial_result =
            runStartupTrial(motors, u_left, u_right);

        printTrialResult(trial, trial_result);

        if (trial_result.both_moved) {
            ++result.successful_trials;

            total_left_time += trial_result.left.first_motion_time_s;
            total_right_time += trial_result.right.first_motion_time_s;
            total_difference += trial_result.start_time_difference_s;
        }
        else if (stop_on_first_failure) {
            std::cout << "  Candidate failed. Skipping remaining trials.\n";
            break;
        }
    }

    result.reliable = (result.successful_trials == NUM_TRIALS);

    if (result.reliable) {
        result.avg_left_start_time_s =
            total_left_time / static_cast<double>(NUM_TRIALS);

        result.avg_right_start_time_s =
            total_right_time / static_cast<double>(NUM_TRIALS);

        result.avg_start_difference_s =
            total_difference / static_cast<double>(NUM_TRIALS);
    }

    coastAll(motors);

    return result;
}

// ============================================================
// AUTOMATIC SEARCH
// ============================================================

void automaticSearch(RobotMotors& motors,
                     double left_start,
                     double right_start,
                     int direction)
{
    std::cout << "\n========================================\n"
              << (direction > 0 ? "FORWARD" : "REVERSE")
              << " AUTOMATIC STARTUP SEARCH\n"
              << "========================================\n"
              << "Start magnitudes:\n"
              << "  left  = " << std::fixed << std::setprecision(2)
              << left_start << '\n'
              << "  right = " << right_start << '\n'
              << "Step = " << SEARCH_STEP << '\n'
              << "Reliable = " << NUM_TRIALS << "/"
              << NUM_TRIALS << " successful starts\n"
              << "Maximum command magnitude = "
              << MAX_SEARCH_COMMAND << '\n'
              << "========================================\n";

    double left_magnitude = left_start;
    double right_magnitude = right_start;

    int candidate_number = 1;

    while (keep_running) {
        if (left_magnitude > MAX_SEARCH_COMMAND + 1e-9 ||
            right_magnitude > MAX_SEARCH_COMMAND + 1e-9) {

            std::cout << "\n========================================\n"
                      << "SEARCH STOPPED\n"
                      << "No reliable pair found before command "
                      << MAX_SEARCH_COMMAND << ".\n"
                      << "========================================\n";

            return;
        }

        const double u_left = direction * left_magnitude;
        const double u_right = direction * right_magnitude;

        std::cout << "\n----------------------------------------\n"
                  << "Candidate " << candidate_number << '\n'
                  << "u_left  = "
                  << std::fixed
                  << std::setprecision(2)
                  << u_left << '\n'
                  << "u_right = "
                  << u_right << '\n'
                  << "----------------------------------------\n";

        const CandidateResult result =
            testCandidate(motors, u_left, u_right, true);

        if (!keep_running) {
            return;
        }

        if (result.reliable) {
            std::cout << "\n========================================\n"
                      << "*** LOWEST TESTED RELIABLE PAIR ***\n"
                      << "========================================\n"
                      << std::fixed
                      << std::setprecision(3)
                      << "u_left  = " << u_left << '\n'
                      << "u_right = " << u_right << '\n'
                      << "success = " << NUM_TRIALS << "/"
                      << NUM_TRIALS << '\n'
                      << "avg tL  = "
                      << result.avg_left_start_time_s << " s\n"
                      << "avg tR  = "
                      << result.avg_right_start_time_s << " s\n"
                      << "avg |tL-tR| = "
                      << result.avg_start_difference_s << " s\n"
                      << "========================================\n";

            return;
        }

        std::cout << "\nCandidate not reliable.\n"
                  << "Increasing both magnitudes by "
                  << SEARCH_STEP << "...\n";

        left_magnitude += SEARCH_STEP;
        right_magnitude += SEARCH_STEP;

        ++candidate_number;
    }
}

// ============================================================
// MANUAL TEST
// ============================================================

void manualTest(RobotMotors& motors,
                double u_left,
                double u_right)
{
    std::cout << "\n========================================\n"
              << "MANUAL STARTUP TEST\n"
              << "========================================\n"
              << std::fixed
              << std::setprecision(3)
              << "u_left  = " << u_left << '\n'
              << "u_right = " << u_right << '\n'
              << "========================================\n";

    const CandidateResult result =
        testCandidate(motors, u_left, u_right, false);

    std::cout << "\nSuccessful starts = "
              << result.successful_trials
              << "/"
              << NUM_TRIALS
              << '\n';

    if (result.reliable) {
        std::cout << "*** RELIABLE STARTUP PAIR ***\n"
                  << "Average left start time  = "
                  << result.avg_left_start_time_s << " s\n"
                  << "Average right start time = "
                  << result.avg_right_start_time_s << " s\n"
                  << "Average |tL-tR|          = "
                  << result.avg_start_difference_s << " s\n";
    }
    else {
        std::cout << "*** STARTUP PAIR NOT RELIABLE ***\n";
    }
}

// ============================================================
// MAIN
// ============================================================

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const MotorConfig left = sparcbot::config::leftMotor();
    const MotorConfig right = sparcbot::config::rightMotor();

    try {
        RobotMotors motors(left, right);

        coastAll(motors);

        std::cout
            << "\n========================================\n"
            << " SPARCBot Automatic Startup Search\n"
            << "========================================\n\n"
            << "Each startup pulse lasts at most "
            << MAX_PULSE_MS << " ms.\n"
            << "Both motors coast as soon as both wheels\n"
            << "reach " << MOTION_THRESHOLD_COUNTS
            << " encoder counts.\n\n"
            << "Automatic search requires "
            << NUM_TRIALS << "/" << NUM_TRIALS
            << " successful starts.\n"
            << "If a candidate fails, both command\n"
            << "magnitudes increase by "
            << SEARCH_STEP << ".\n\n"
            << "Ctrl+C stops the test.\n\n";

        while (keep_running) {
            std::cout
                << "----------------------------------------\n"
                << "f : automatic FORWARD search\n"
                << "r : automatic REVERSE search\n"
                << "t : manually test a command pair\n"
                << "q : quit\n"
                << "----------------------------------------\n"
                << "> "
                << std::flush;

            char command;
            std::cin >> command;

            if (!std::cin) {
                break;
            }

            if (command == 'q' || command == 'Q') {
                break;
            }

            if (command == 'f' || command == 'F') {
                std::cout
                    << "\nForward search will begin at:\n"
                    << "  left  = +" << FORWARD_LEFT_START << '\n'
                    << "  right = +" << FORWARD_RIGHT_START << '\n'
                    << "\nMake sure the robot has clear space.\n"
                    << "Press ENTER to start...";

                std::cin.ignore(
                    std::numeric_limits<std::streamsize>::max(),
                    '\n');

                std::cin.get();

                automaticSearch(
                    motors,
                    FORWARD_LEFT_START,
                    FORWARD_RIGHT_START,
                    +1);
            }

            else if (command == 'r' || command == 'R') {
                std::cout
                    << "\nReverse search will begin at:\n"
                    << "  left  = -" << REVERSE_LEFT_START << '\n'
                    << "  right = -" << REVERSE_RIGHT_START << '\n'
                    << "\nMake sure the robot has clear space.\n"
                    << "Press ENTER to start...";

                std::cin.ignore(
                    std::numeric_limits<std::streamsize>::max(),
                    '\n');

                std::cin.get();

                automaticSearch(
                    motors,
                    REVERSE_LEFT_START,
                    REVERSE_RIGHT_START,
                    -1);
            }

            else if (command == 't' || command == 'T') {
                double u_left = 0.0;
                double u_right = 0.0;

                std::cout << "Enter u_left  [-1, 1]: ";
                std::cin >> u_left;

                std::cout << "Enter u_right [-1, 1]: ";
                std::cin >> u_right;

                if (!std::cin) {
                    break;
                }

                if (std::abs(u_left) > 1.0 ||
                    std::abs(u_right) > 1.0) {

                    std::cout << "Commands must be within [-1, 1].\n";
                    continue;
                }

                std::cout
                    << "\nMake sure the robot has clear space.\n"
                    << "Press ENTER to start...";

                std::cin.ignore(
                    std::numeric_limits<std::streamsize>::max(),
                    '\n');

                std::cin.get();

                manualTest(
                    motors,
                    u_left,
                    u_right);
            }

            else {
                std::cout << "Use f, r, t, or q.\n";
            }

            coastAll(motors);
        }

        coastAll(motors);

        std::cout << "\nDone.\n";
    }

    catch (const std::exception& e) {
        std::cerr << "\nERROR: "
                  << e.what()
                  << '\n';

        return 1;
    }

    return 0;
}