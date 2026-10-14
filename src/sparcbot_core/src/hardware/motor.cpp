#include "sparcbot/hardware/motor.hpp"
#include <gpiod.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <poll.h>
#include <stdexcept>
#include <thread>
#include <unistd.h>

bool HardwarePWM::exists(const std::string& path) {
    return ::access(path.c_str(), F_OK) == 0;
}

void HardwarePWM::writeFile(const std::string& path, const std::string& value) {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("Could not open " + path + " (run as root / check PWM overlay)");
    f << value;
    if (!f) throw std::runtime_error("Could not write to " + path);
}

void HardwarePWM::waitForPath(const std::string& path, int timeout_ms) {
    auto start = std::chrono::steady_clock::now();
    while (!exists(path)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed > timeout_ms) throw std::runtime_error("Timed out waiting for " + path);
    }
}

HardwarePWM::HardwarePWM(int chip, int channel, int frequency_hz) {
    if (frequency_hz <= 0) throw std::invalid_argument("PWM frequency must be > 0");
    const std::string chip_base = "/sys/class/pwm/pwmchip" + std::to_string(chip);
    base_ = chip_base + "/pwm" + std::to_string(channel);
    if (!exists(chip_base)) throw std::runtime_error(chip_base + " does not exist. Check dtoverlay=pwm-2chan.");

    if (!exists(base_)) {
        writeFile(chip_base + "/export", std::to_string(channel));
        waitForPath(base_);
    }

    writeFile(base_ + "/enable", "0");
    period_ns_ = static_cast<long long>(std::llround(1.0e9 / static_cast<double>(frequency_hz)));
    if (exists(base_ + "/polarity")) writeFile(base_ + "/polarity", "normal");
    writeFile(base_ + "/period", std::to_string(period_ns_));
    writeFile(base_ + "/duty_cycle", "0");
    writeFile(base_ + "/enable", "1");
}

HardwarePWM::~HardwarePWM() {
    try {
        if (exists(base_ + "/duty_cycle")) writeFile(base_ + "/duty_cycle", "0");
        if (exists(base_ + "/enable")) writeFile(base_ + "/enable", "0");
    } catch (...) {}
}

void HardwarePWM::setDuty(double duty) {
    duty = std::clamp(duty, 0.0, 1.0);
    long long duty_ns = (duty >= 1.0)
        ? std::max<long long>(0, period_ns_ - 1)
        : static_cast<long long>(std::llround(duty * static_cast<double>(period_ns_)));
    writeFile(base_ + "/duty_cycle", std::to_string(duty_ns));
}

Motor::Motor(const MotorConfig& cfg)
    : cfg_(cfg), pwm_(cfg.pwm_chip, cfg.pwm_channel, cfg.pwm_frequency_hz) {
    if ((cfg_.motor_sign != 1 && cfg_.motor_sign != -1) ||
        (cfg_.encoder_sign != 1 && cfg_.encoder_sign != -1))
        throw std::invalid_argument("motor_sign and encoder_sign must be +1 or -1");
    if (cfg_.counts_per_output_rev <= 0.0)
        throw std::invalid_argument("counts_per_output_rev must be > 0");
    setupGPIO();
    coast();
    startEncoder();
}

Motor::~Motor() {
    try { coast(); } catch (...) {}
    stopEncoder();
    if (dir_line_) gpiod_line_release(dir_line_);
    if (ps_line_) gpiod_line_release(ps_line_);
    if (enc_a_line_) gpiod_line_release(enc_a_line_);
    if (enc_b_line_) gpiod_line_release(enc_b_line_);
    if (chip_) gpiod_chip_close(chip_);
}

void Motor::setupGPIO() {
    chip_ = gpiod_chip_open_by_name("gpiochip0");
    if (!chip_) throw std::runtime_error("Could not open gpiochip0");

    dir_line_ = gpiod_chip_get_line(chip_, cfg_.gpio_dir);
    ps_line_ = gpiod_chip_get_line(chip_, cfg_.gpio_ps);
    enc_a_line_ = gpiod_chip_get_line(chip_, cfg_.gpio_enc_a);
    enc_b_line_ = gpiod_chip_get_line(chip_, cfg_.gpio_enc_b);
    if (!dir_line_ || !ps_line_ || !enc_a_line_ || !enc_b_line_)
        throw std::runtime_error("Could not get GPIO lines for " + cfg_.name);

    const std::string consumer = "motor_" + cfg_.name;
    if (gpiod_line_request_output(dir_line_, consumer.c_str(), 0) < 0)
        throw std::runtime_error("Could not request DIR GPIO for " + cfg_.name);
    if (gpiod_line_request_output(ps_line_, consumer.c_str(), 0) < 0)
        throw std::runtime_error("Could not request PS GPIO for " + cfg_.name);
    if (gpiod_line_request_both_edges_events(enc_a_line_, (consumer + "_encA").c_str()) < 0)
        throw std::runtime_error("Could not request encoder A for " + cfg_.name);
    if (gpiod_line_request_both_edges_events(enc_b_line_, (consumer + "_encB").c_str()) < 0)
        throw std::runtime_error("Could not request encoder B for " + cfg_.name);
}

void Motor::setPS(bool enabled) {
    if (gpiod_line_set_value(ps_line_, enabled ? 1 : 0) < 0)
        throw std::runtime_error("Failed to set PS for " + cfg_.name);
}

void Motor::setDirectionPhysical(int logical_dir) {
    const int physical_dir = logical_dir * cfg_.motor_sign;
    const int dir_value = (physical_dir > 0) ? 0 : 1;  // +physical => INB LOW
    if (gpiod_line_set_value(dir_line_, dir_value) < 0)
        throw std::runtime_error("Failed to set DIR for " + cfg_.name);
}

void Motor::setCommand(double u){
    u = std::clamp(u, -1.0, 1.0);

    // No command -> coast
    if (std::abs(u) < 1e-9) {
        coast();
        return;
    }

    const int new_dir = (u > 0.0) ? +1 : -1;
    const double duty = std::abs(u);

    if (last_dir_ != 0 && new_dir != last_dir_) {
        pwm_.setDuty(0.0);
    }

    setDirectionPhysical(new_dir);
    setPS(true);
    pwm_.setDuty(duty);

    last_dir_ = new_dir;
    command_.store(u, std::memory_order_relaxed);
}

void Motor::coast() {
    pwm_.setDuty(0.0);
    setPS(false);
    command_.store(0.0, std::memory_order_relaxed);
    last_dir_ = 0;
}

void Motor::brake() {
    pwm_.setDuty(0.0);
    setPS(true);
    command_.store(0.0, std::memory_order_relaxed);
    last_dir_ = 0;
}

int64_t Motor::getEncoderCount() const {
    return encoder_count_.load(std::memory_order_relaxed);
}

double Motor::getPositionRad() const {
    constexpr double TWO_PI = 6.2831853071795864769;
    return static_cast<double>(getEncoderCount()) * (TWO_PI / cfg_.counts_per_output_rev);
}

double Motor::getCommand() const {
    return command_.load(std::memory_order_relaxed);
}

MotorState Motor::getState() const {
    return {getEncoderCount(), getPositionRad(), getCommand()};
}

void Motor::resetEncoder(int64_t new_count) {
    encoder_count_.store(new_count, std::memory_order_relaxed);
}

int Motor::quadratureDelta(int old_state, int new_state) {
    static constexpr int8_t table[16] = {
         0, +1, -1,  0,
        -1,  0,  0, +1,
        +1,  0,  0, -1,
         0, -1, +1,  0
    };
    return table[((old_state & 3) << 2) | (new_state & 3)];
}

void Motor::startEncoder() {
    encoder_running_.store(true);
    encoder_thread_ = std::thread(&Motor::encoderLoop, this);
}

void Motor::stopEncoder() {
    encoder_running_.store(false);
    if (encoder_thread_.joinable()) encoder_thread_.join();
}

void Motor::encoderLoop() {
    int a = gpiod_line_get_value(enc_a_line_);
    int b = gpiod_line_get_value(enc_b_line_);
    if (a < 0 || b < 0) { a = 0; b = 0; }
    int old_state = (a << 1) | b;

    struct pollfd fds[2] = {
        {gpiod_line_event_get_fd(enc_a_line_), POLLIN, 0},
        {gpiod_line_event_get_fd(enc_b_line_), POLLIN, 0}
    };

    while (encoder_running_.load()) {
        const int rc = ::poll(fds, 2, 50);
        if (rc <= 0) continue;

        if (fds[0].revents & POLLIN) {
            struct gpiod_line_event ev;
            (void)gpiod_line_event_read(enc_a_line_, &ev);
        }
        if (fds[1].revents & POLLIN) {
            struct gpiod_line_event ev;
            (void)gpiod_line_event_read(enc_b_line_, &ev);
        }

        a = gpiod_line_get_value(enc_a_line_);
        b = gpiod_line_get_value(enc_b_line_);
        if (a < 0 || b < 0) continue;

        const int new_state = (a << 1) | b;
        const int delta = quadratureDelta(old_state, new_state);
        old_state = new_state;

        if (delta != 0)
            encoder_count_.fetch_add(cfg_.encoder_sign * delta, std::memory_order_relaxed);
    }
}
