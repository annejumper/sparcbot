"""Keyboard teleop for SPARCBot.

Instead of driving the motors directly it publishes /cmd_vel and calls the controller / driver
services, so it needs the robot stack running:

    ros2 launch sparcbot_bringup robot.launch.py      # terminal 1
    ros2 run sparcbot_teleop keyboard_teleop          # terminal 2

Run it with `ros2 run` (not from a launch file): it needs the terminal's keyboard.
"""

import fcntl
import os
import sys
import termios

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sparcbot_interfaces.msg import WheelControllerDebug
from std_srvs.srv import Trigger

MIN_LINEAR_SPEED_M_S = 0.05
MAX_LINEAR_SPEED_M_S = 0.20
LINEAR_SPEED_STEP_M_S = 0.05

MIN_ANGULAR_SPEED_RAD_S = 0.25
MAX_ANGULAR_SPEED_RAD_S = 1.5
ANGULAR_SPEED_STEP_RAD_S = 0.25

PRINT_PERIOD_S = 0.2

HELP = """
Keyboard Teleop With Wheel PI + Kinematics
==========================================
  w : forward
  s : backward
  a : rotate left
  d : rotate right
  space : stop/coast
  b : brake
  + / = : increase linear speed
  -     : decrease linear speed
  ]     : increase angular speed
  [     : decrease angular speed
  r     : reset encoders/controllers
  q     : quit
"""


class RawTerminal:
    """Non-blocking, no-echo keyboard input; restores the terminal on exit."""

    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_term = termios.tcgetattr(self.fd)
        raw = termios.tcgetattr(self.fd)
        raw[3] &= ~(termios.ICANON | termios.ECHO)
        raw[6][termios.VMIN] = 0
        raw[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, raw)
        self.old_flags = fcntl.fcntl(self.fd, fcntl.F_GETFL)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags | os.O_NONBLOCK)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSANOW, self.old_term)
        fcntl.fcntl(self.fd, fcntl.F_SETFL, self.old_flags)

    def read_key(self):
        try:
            data = os.read(self.fd, 1)
        except BlockingIOError:
            return None
        return data.decode(errors='ignore') if data else None


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.coast_client = self.create_client(Trigger, 'wheel_controller/coast')
        self.brake_client = self.create_client(Trigger, 'wheel_controller/brake')
        self.reset_controller_client = self.create_client(Trigger, 'wheel_controller/reset')
        self.reset_encoders_client = self.create_client(Trigger, 'sparcbot_driver/reset_encoders')

        self.create_subscription(
            WheelControllerDebug, 'wheel_controller/debug', self.on_debug, 10)
        self.create_subscription(TwistStamped, 'body_velocity', self.on_body_velocity, 10)

        self.linear_speed_m_s = 0.10
        self.angular_speed_rad_s = 1.0

        self.linear_target_m_s = 0.0
        self.angular_target_rad_s = 0.0

        self.debug = None
        self.body_velocity = None

        self.create_timer(PRINT_PERIOD_S, self.print_status)

    # ------------------------------------------------------------------

    def on_debug(self, msg):
        self.debug = msg

    def on_body_velocity(self, msg):
        self.body_velocity = msg

    def call(self, client, name):
        if not client.service_is_ready():
            self.get_logger().warn(f'{name} service not available (is the robot launch running?)')
            return None
        return client.call_async(Trigger.Request())

    def drive(self, linear_m_s, angular_rad_s):
        self.linear_target_m_s = linear_m_s
        self.angular_target_rad_s = angular_rad_s
        msg = Twist()
        msg.linear.x = linear_m_s
        msg.angular.z = angular_rad_s
        self.cmd_vel_pub.publish(msg)

    def coast(self):
        self.linear_target_m_s = 0.0
        self.angular_target_rad_s = 0.0
        return self.call(self.coast_client, 'wheel_controller/coast')

    # ------------------------------------------------------------------

    def handle_key(self, key):
        """Return False when the user asked to quit."""
        if key in ('w', 'W'):
            self.drive(+self.linear_speed_m_s, 0.0)
        elif key in ('s', 'S'):
            self.drive(-self.linear_speed_m_s, 0.0)
        elif key in ('a', 'A'):
            self.drive(0.0, +self.angular_speed_rad_s)
        elif key in ('d', 'D'):
            self.drive(0.0, -self.angular_speed_rad_s)
        elif key == ' ':
            self.coast()
        elif key in ('b', 'B'):
            self.linear_target_m_s = 0.0
            self.angular_target_rad_s = 0.0
            self.call(self.brake_client, 'wheel_controller/brake')
        elif key in ('+', '='):
            self.linear_speed_m_s = min(
                MAX_LINEAR_SPEED_M_S, self.linear_speed_m_s + LINEAR_SPEED_STEP_M_S)
            print(f'\nLinear speed = {self.linear_speed_m_s:g} m/s')
        elif key in ('-', '_'):
            self.linear_speed_m_s = max(
                MIN_LINEAR_SPEED_M_S, self.linear_speed_m_s - LINEAR_SPEED_STEP_M_S)
            print(f'\nLinear speed = {self.linear_speed_m_s:g} m/s')
        elif key == ']':
            self.angular_speed_rad_s = min(
                MAX_ANGULAR_SPEED_RAD_S, self.angular_speed_rad_s + ANGULAR_SPEED_STEP_RAD_S)
            print(f'\nAngular speed = {self.angular_speed_rad_s:g} rad/s')
        elif key == '[':
            self.angular_speed_rad_s = max(
                MIN_ANGULAR_SPEED_RAD_S, self.angular_speed_rad_s - ANGULAR_SPEED_STEP_RAD_S)
            print(f'\nAngular speed = {self.angular_speed_rad_s:g} rad/s')
        elif key in ('r', 'R'):
            self.call(self.reset_encoders_client, 'sparcbot_driver/reset_encoders')
            self.call(self.reset_controller_client, 'wheel_controller/reset')
            print('\nEncoders and controllers reset.')
        elif key in ('q', 'Q'):
            return False
        return True

    def print_status(self):
        if self.debug is None:
            return

        left = self.debug.left
        right = self.debug.right
        v = self.body_velocity.twist.linear.x if self.body_velocity else 0.0
        w = self.body_velocity.twist.angular.z if self.body_velocity else 0.0

        def f(x):
            return f'{x:7.4f}'

        print(
            f'CMD BODY: v={f(self.linear_target_m_s)} w={f(self.angular_target_rad_s)}'
            f' | TARGET WHEELS: L={f(left.target_m_s)} R={f(right.target_m_s)}'
            f' | MEAS WHEELS: L={f(left.measured_m_s)} R={f(right.measured_m_s)}'
            f' | ERROR: L={f(left.error_m_s)} R={f(right.error_m_s)}'
            f' | EST BODY: v={f(v)} w={f(w)}'
            f' | PWM: L={f(left.command)} R={f(right.command)}'
        )


def main(args=None):
    # Handle Ctrl-C ourselves so ROS is still up to send the final coast,
    # exactly like the original app (which coasted the motors on exit).
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleop()
    print(HELP)

    try:
        with RawTerminal() as terminal:
            running = True
            while running and rclpy.ok():
                key = terminal.read_key()
                if key is not None:
                    running = node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.01)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            future = node.coast()
            if future is not None:
                rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)
            print('\nStopped. Both motors are coasting.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
