"""wheel_controller node: runs the left/right WheelVelocityController.

Given ROS plumbing.

Every /wheel_states message (100 Hz, from sparcbot_driver):
  * VELOCITY mode: compute left/right commands with the measured dt and
    publish them on /wheel_cmd as duty.
  * COAST / BRAKE mode: publish a coast / brake command.

Modes (same behaviour as the original keyboard_teleop_with_wheelpid_kinematics):
  * starts in COAST
  * any /wheel_targets message -> VELOCITY (holds the last target)
  * ~/coast, ~/brake  -> zero targets, reset both controllers, coast / brake
  * ~/reset           -> reset both controllers (mode unchanged)

Topics
  sub  wheel_targets           sparcbot_interfaces/WheelVelocities
  sub  wheel_states            sparcbot_interfaces/WheelStates
  pub  wheel_cmd               sparcbot_interfaces/WheelCommand
  pub  ~/debug                 sparcbot_interfaces/WheelControllerDebug
Services
  ~/coast  ~/brake  ~/reset    std_srvs/Trigger
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from sparcbot_interfaces.msg import (
    WheelCommand,
    WheelControllerDebug,
    WheelControllerOutput,
    WheelStates,
    WheelVelocities,
)
from std_srvs.srv import Trigger

from sparcbot_control.feedforward import WheelFeedforwardConfig
from sparcbot_control.wheel_velocity_controller import (
    PIDGains,
    WheelVelocityController,
    WheelVelocityControllerOutput,
)

FEEDFORWARD_KEYS = (
    'forward_static',
    'forward_velocity_gain',
    'reverse_static',
    'reverse_velocity_gain',
    'forward_breakaway',
    'reverse_breakaway',
    'stopped_velocity_threshold',
    'max_command',
    'zero_velocity_threshold',
)

MODE_VELOCITY = WheelControllerDebug.MODE_VELOCITY
MODE_COAST = WheelControllerDebug.MODE_COAST
MODE_BRAKE = WheelControllerDebug.MODE_BRAKE


class WheelControllerNode(Node):

    def __init__(self):
        super().__init__('wheel_controller')

        self.left_controller = WheelVelocityController(
            self._feedforward_config('left'), self._gains('left'))
        self.right_controller = WheelVelocityController(
            self._feedforward_config('right'), self._gains('right'))

        self.mode = MODE_COAST
        self.left_target_m_s = 0.0
        self.right_target_m_s = 0.0

        control_qos = QoSProfile(depth=1)

        self.cmd_pub = self.create_publisher(WheelCommand, 'wheel_cmd', control_qos)
        self.debug_pub = self.create_publisher(WheelControllerDebug, '~/debug', 10)

        self.create_subscription(WheelVelocities, 'wheel_targets', self.on_targets, 10)
        self.create_subscription(WheelStates, 'wheel_states', self.on_wheel_states, control_qos)

        self.create_service(Trigger, '~/coast', self.on_coast)
        self.create_service(Trigger, '~/brake', self.on_brake)
        self.create_service(Trigger, '~/reset', self.on_reset)

        self._not_implemented_reported = False

        self.get_logger().info('wheel_controller ready (COAST until the first wheel target).')

    # ------------------------------------------------------------------
    # Parameters (all required: they come from sparcbot.yaml)
    # ------------------------------------------------------------------

    def _required_double(self, name: str) -> float:
        self.declare_parameter(name, Parameter.Type.DOUBLE)
        value = self.get_parameter(name).value
        if value is None:
            raise RuntimeError(f"Missing parameter '{name}' (check config/sparcbot.yaml)")
        return float(value)

    def _gains(self, wheel: str) -> PIDGains:
        return PIDGains(
            kp=self._required_double(f'{wheel}.kp'),
            ki=self._required_double(f'{wheel}.ki'),
            kd=self._required_double(f'{wheel}.kd'),
        )

    def _feedforward_config(self, wheel: str) -> WheelFeedforwardConfig:
        values = {
            key: self._required_double(f'{wheel}.feedforward.{key}')
            for key in FEEDFORWARD_KEYS
        }
        return WheelFeedforwardConfig(**values)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def on_targets(self, msg: WheelVelocities) -> None:
        self.left_target_m_s = msg.left_m_s
        self.right_target_m_s = msg.right_m_s
        self.mode = MODE_VELOCITY

    def on_wheel_states(self, msg: WheelStates) -> None:
        dt_s = msg.dt_s
        left = WheelVelocityControllerOutput()
        right = WheelVelocityControllerOutput()

        cmd = WheelCommand()
        cmd.header.stamp = msg.header.stamp

        if self.mode == MODE_VELOCITY:
            try:
                left = self.left_controller.compute(
                    self.left_target_m_s, msg.left.velocity_m_s, dt_s)
                right = self.right_controller.compute(
                    self.right_target_m_s, msg.right.velocity_m_s, dt_s)
            except NotImplementedError as e:
                if not self._not_implemented_reported:
                    self.get_logger().error(f'{e} - motors will coast.')
                    self._not_implemented_reported = True
                cmd.mode = WheelCommand.MODE_COAST
                self.cmd_pub.publish(cmd)
                return

            cmd.mode = WheelCommand.MODE_DUTY
            cmd.left = float(left.command)
            cmd.right = float(right.command)

        elif self.mode == MODE_BRAKE:
            cmd.mode = WheelCommand.MODE_BRAKE

        else:
            cmd.mode = WheelCommand.MODE_COAST

        self.cmd_pub.publish(cmd)
        self._publish_debug(msg, left, right)

    def _publish_debug(self, states, left, right) -> None:
        debug = WheelControllerDebug()
        debug.header.stamp = states.header.stamp
        debug.mode = self.mode
        debug.dt_s = states.dt_s
        debug.left = self._output_msg(self.left_target_m_s, states.left.velocity_m_s, left)
        debug.right = self._output_msg(self.right_target_m_s, states.right.velocity_m_s, right)
        self.debug_pub.publish(debug)

    @staticmethod
    def _output_msg(target, measured, output) -> WheelControllerOutput:
        msg = WheelControllerOutput()
        msg.target_m_s = float(target)
        msg.measured_m_s = float(measured)
        msg.error_m_s = float(target - measured)
        msg.feedforward_command = float(output.feedforward_command)
        msg.feedback_command = float(output.feedback_command)
        msg.command = float(output.command)
        return msg

    def _stop(self, mode: int) -> None:
        self.mode = mode
        self.left_target_m_s = 0.0
        self.right_target_m_s = 0.0
        self.left_controller.reset()
        self.right_controller.reset()

    def on_coast(self, request, response):
        self._stop(MODE_COAST)
        response.success = True
        response.message = 'Coasting.'
        return response

    def on_brake(self, request, response):
        self._stop(MODE_BRAKE)
        response.success = True
        response.message = 'Braking.'
        return response

    def on_reset(self, request, response):
        self.left_controller.reset()
        self.right_controller.reset()
        response.success = True
        response.message = 'Controllers reset.'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
