#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

struct MotorConfig
{
    std::string name;
    int gpio_dir;
    int gpio_ps;
    int gpio_enc_a;
    int gpio_enc_b;
    int pwm_chip;
    int pwm_channel;
    int pwm_frequency_hz;
    double counts_per_output_rev;
    int motor_sign;
    int encoder_sign;
};

struct MotorState {
    int64_t counts = 0;
    double position_rad = 0.0;
    double command = 0.0;
};

class HardwarePWM {
public:
    HardwarePWM(int chip, int channel, int frequency_hz);
    ~HardwarePWM();
    HardwarePWM(const HardwarePWM&) = delete;
    HardwarePWM& operator=(const HardwarePWM&) = delete;
    void setDuty(double duty_0_to_1);

private:
    std::string base_;
    long long period_ns_ = 0;
    static bool exists(const std::string& path);
    static void writeFile(const std::string& path, const std::string& value);
    static void waitForPath(const std::string& path, int timeout_ms = 1000);
};

class Motor {
public:
    explicit Motor(const MotorConfig& cfg);
    ~Motor();
    Motor(const Motor&) = delete;
    Motor& operator=(const Motor&) = delete;

    void setCommand(double u);   // normalized [-1, 1]
    void coast();                // PS LOW -> open outputs
    void brake();                // PS HIGH + PWM=0 -> short brake

    int64_t getEncoderCount() const;
    double getPositionRad() const;
    double getCommand() const;
    MotorState getState() const;
    void resetEncoder(int64_t new_count = 0);

    const MotorConfig& config() const { return cfg_; }

private:
    MotorConfig cfg_;
    HardwarePWM pwm_;

    struct gpiod_chip* chip_ = nullptr;
    struct gpiod_line* dir_line_ = nullptr;
    struct gpiod_line* ps_line_ = nullptr;
    struct gpiod_line* enc_a_line_ = nullptr;
    struct gpiod_line* enc_b_line_ = nullptr;

    std::atomic<int64_t> encoder_count_{0};
    std::atomic<double> command_{0.0};
    std::atomic<bool> encoder_running_{false};
    std::thread encoder_thread_;
    int last_dir_ = 0;

    void setupGPIO();
    void startEncoder();
    void stopEncoder();
    void encoderLoop();
    void setDirectionPhysical(int logical_dir);
    void setPS(bool enabled);
    static int quadratureDelta(int old_state, int new_state);
};
