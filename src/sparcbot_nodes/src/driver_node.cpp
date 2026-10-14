// sparcbot_driver
//
// Thin ROS 2 wrapper around the sparcbot_core hardware + estimation code.
// No control logic lives here.
//
// Every control period (control_rate_hz):
//   1. read both encoders                                (RobotMotors)
//   2. update the wheel-state estimators with the real dt (WheelStateEstimator)
//   3. publish /wheel_states (includes dt_s for the controller)
//
// Whenever a /wheel_cmd arrives it is applied to the motors immediately.
// If no /wheel_cmd arrives for cmd_timeout_s, the motors coast (safety watchdog).
//
// Topics
//   pub  wheel_states   sparcbot_interfaces/WheelStates
//   sub  wheel_cmd      sparcbot_interfaces/WheelCommand
// Services
//   ~/reset_encoders    std_srvs/Trigger   zero encoders + estimators

#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "sparcbot/estimation/wheel_state_estimator.hpp"
#include "sparcbot/hardware/motor.hpp"
#include "sparcbot/hardware/robot_motors.hpp"
#include "sparcbot/kinematics/wheel_geometry.hpp"
#include "sparcbot_interfaces/msg/wheel_command.hpp"
#include "sparcbot_interfaces/msg/wheel_states.hpp"

using sparcbot_interfaces::msg::WheelCommand;
using sparcbot_interfaces::msg::WheelStates;

namespace {

using SteadyClock = std::chrono::steady_clock;

class SparcbotDriverNode : public rclcpp::Node {
public:
    SparcbotDriverNode()
        : Node("sparcbot_driver")
    {
        // ---------------------------------------------------------------
        // Parameters (all required: they come from sparcbot.yaml)
        // ---------------------------------------------------------------
        const double control_rate_hz = declare_parameter<double>("control_rate_hz");
        cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s");

        if (control_rate_hz <= 0.0) {
            throw std::invalid_argument("control_rate_hz must be > 0");
        }

        const double counts_per_rev = declare_parameter<double>("counts_per_rev");

        WheelStateEstimatorConfig estimator_config;
        estimator_config.counts_per_rev = counts_per_rev;
        estimator_config.velocity_cutoff_hz = declare_parameter<double>("velocity_cutoff_hz");

        WheelGeometryConfig geometry_config;
        geometry_config.radius_m = declare_parameter<double>("wheel_radius_m");

        const MotorConfig left_config = declareMotorConfig("left_motor", counts_per_rev);
        const MotorConfig right_config = declareMotorConfig("right_motor", counts_per_rev);

        // ---------------------------------------------------------------
        // sparcbot_core objects
        // ---------------------------------------------------------------
        motors_ = std::make_unique<RobotMotors>(left_config, right_config);
        left_estimator_ = std::make_unique<WheelStateEstimator>(estimator_config);
        right_estimator_ = std::make_unique<WheelStateEstimator>(estimator_config);
        geometry_ = std::make_unique<WheelGeometry>(geometry_config);

        // Start exactly like the original teleop: zeroed encoders, coasting.
        motors_->resetEncoders();
        left_estimator_->reset();
        right_estimator_->reset();
        motors_->coast();
        mode_ = WheelCommand::MODE_COAST;

        // ---------------------------------------------------------------
        // ROS interfaces
        // ---------------------------------------------------------------
        const auto qos = rclcpp::QoS(rclcpp::KeepLast(1));

        states_pub_ = create_publisher<WheelStates>("wheel_states", qos);

        cmd_sub_ = create_subscription<WheelCommand>(
            "wheel_cmd", qos,
            [this](const WheelCommand& msg) { onCommand(msg); });

        reset_srv_ = create_service<std_srvs::srv::Trigger>(
            "~/reset_encoders",
            [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                   std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
                motors_->resetEncoders();
                left_estimator_->reset();
                right_estimator_->reset();
                response->success = true;
                response->message = "Encoders and estimators reset.";
                RCLCPP_INFO(get_logger(), "Encoders and estimators reset.");
            });

        previous_time_ = SteadyClock::now();
        last_cmd_time_ = previous_time_;

        // Wall timers run on the steady clock, like the original control loop.
        timer_ = create_wall_timer(
            std::chrono::duration<double>(1.0 / control_rate_hz),
            [this]() { onTimer(); });

        RCLCPP_INFO(get_logger(), "SPARCBot driver running at %.1f Hz (cmd timeout %.3f s).",
                    control_rate_hz, cmd_timeout_s_);
    }

    ~SparcbotDriverNode() override
    {
        if (motors_) {
            motors_->coast();
        }
    }

private:
    MotorConfig declareMotorConfig(const std::string& prefix, double counts_per_rev)
    {
        auto get_int = [this, &prefix](const std::string& key) {
            return static_cast<int>(declare_parameter<int64_t>(prefix + "." + key));
        };

        MotorConfig config;
        config.name = declare_parameter<std::string>(prefix + ".name");
        config.gpio_dir = get_int("gpio_dir");
        config.gpio_ps = get_int("gpio_ps");
        config.gpio_enc_a = get_int("gpio_enc_a");
        config.gpio_enc_b = get_int("gpio_enc_b");
        config.pwm_chip = get_int("pwm_chip");
        config.pwm_channel = get_int("pwm_channel");
        config.pwm_frequency_hz = get_int("pwm_frequency_hz");
        config.counts_per_output_rev = counts_per_rev;
        config.motor_sign = get_int("motor_sign");
        config.encoder_sign = get_int("encoder_sign");
        return config;
    }

    void onTimer()
    {
        const auto now = SteadyClock::now();
        const double dt_s = std::chrono::duration<double>(now - previous_time_).count();

        if (dt_s <= 0.0) {
            return;
        }
        previous_time_ = now;

        // Safety watchdog: a crashed or stalled controller must not leave
        // the motors running at their last duty.
        if (mode_ == WheelCommand::MODE_DUTY && cmd_timeout_s_ > 0.0) {
            const double since_cmd = std::chrono::duration<double>(now - last_cmd_time_).count();
            if (since_cmd > cmd_timeout_s_) {
                motors_->coast();
                mode_ = WheelCommand::MODE_COAST;
                watchdog_tripped_ = true;
                RCLCPP_WARN(get_logger(),
                            "No wheel_cmd for %.3f s (timeout %.3f s): coasting.",
                            since_cmd, cmd_timeout_s_);
            }
        }

        const RobotMotorState motor_state = motors_->getState();

        const WheelState left = left_estimator_->update(motor_state.left.counts, dt_s);
        const WheelState right = right_estimator_->update(motor_state.right.counts, dt_s);

        WheelStates msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "base_link";
        msg.dt_s = dt_s;
        fillWheel(msg.left, left, motor_state.left.command);
        fillWheel(msg.right, right, motor_state.right.command);

        states_pub_->publish(msg);
    }

    void fillWheel(sparcbot_interfaces::msg::WheelState& out, const WheelState& state, double command) const
    {
        out.counts = state.counts;
        out.position_rad = state.position_rad;
        out.velocity_raw_rad_s = state.velocity_raw_rad_s;
        out.velocity_rad_s = state.velocity_rad_s;
        out.velocity_m_s = geometry_->angularToLinear(state.velocity_rad_s);
        out.command = command;
    }

    void onCommand(const WheelCommand& msg)
    {
        last_cmd_time_ = SteadyClock::now();

        if (watchdog_tripped_) {
            watchdog_tripped_ = false;
            RCLCPP_INFO(get_logger(), "wheel_cmd resumed.");
        }

        switch (msg.mode) {
            case WheelCommand::MODE_DUTY:
                if (!std::isfinite(msg.left) || !std::isfinite(msg.right)) {
                    motors_->coast();
                    mode_ = WheelCommand::MODE_COAST;
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                          "Non-finite wheel command (left=%f, right=%f): coasting.",
                                          msg.left, msg.right);
                    return;
                }
                // Motor::setCommand() performs the final [-1, 1] clamp, as before.
                motors_->setMotorCommands(msg.left, msg.right);
                break;

            case WheelCommand::MODE_COAST:
                motors_->coast();
                break;

            case WheelCommand::MODE_BRAKE:
                motors_->brake();
                break;

            default:
                motors_->coast();
                RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                      "Unknown wheel_cmd mode %u: coasting.", msg.mode);
                mode_ = WheelCommand::MODE_COAST;
                return;
        }

        mode_ = msg.mode;
    }

    std::unique_ptr<RobotMotors> motors_;
    std::unique_ptr<WheelStateEstimator> left_estimator_;
    std::unique_ptr<WheelStateEstimator> right_estimator_;
    std::unique_ptr<WheelGeometry> geometry_;

    rclcpp::Publisher<WheelStates>::SharedPtr states_pub_;
    rclcpp::Subscription<WheelCommand>::SharedPtr cmd_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    double cmd_timeout_s_ = 0.0;
    uint8_t mode_ = WheelCommand::MODE_COAST;
    bool watchdog_tripped_ = false;
    SteadyClock::time_point previous_time_;
    SteadyClock::time_point last_cmd_time_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    int exit_code = 0;
    try {
        auto node = std::make_shared<SparcbotDriverNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("sparcbot_driver"), "%s", e.what());
        exit_code = 1;
    }

    rclcpp::shutdown();
    return exit_code;
}
