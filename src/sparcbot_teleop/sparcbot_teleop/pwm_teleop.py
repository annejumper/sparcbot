"""Open-loop PWM keyboard teleop for SPARCBot (no controller).

Sends duty commands straight to the driver on /wheel_cmd and shows what the
encoders measure on /wheel_states.

    ros2 launch sparcbot_bringup driver.launch.py     # terminal 1
    ros2 run sparcbot_teleop pwm_teleop               # terminal 2

Run it with `ros2 run` (not from a launch file): it needs the terminal's keyboard.
"""

import fcntl
import os
import sys
import termios
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from sparcbot_interfaces.msg import WheelCommand, WheelStates
from std_srvs.srv import Trigger

MIN_CMD = 0.05
MAX_CMD = 0.40
STEP = 0.05

SEND_PERIOD_S = 0.02    # 50 Hz, well inside the driver's cmd_timeout_s
PRINT_PERIOD_S = 0.2

HELP = """
PWM Keyboard Teleop
===================
  w : forward
  s : backward
  a : turn left in place
  d : turn right in place
  space : stop/coast
  b : brake
  + / = : increase drive command
  -     : decrease drive command
  ]     : increase turn command
  [     : decrease turn command
  r     : reset encoder counts
  q     : quit

Open-loop PWM teleop: equal commands may not give equal wheel speeds.
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


class PwmTeleop(Node):

    def __init__(self):
        super().__init__('pwm_teleop')

        self.cmd_pub = self.create_publisher(WheelCommand, 'wheel_cmd', QoSProfile(depth=1))
        self.create_subscription(WheelStates, 'wheel_states', self.on_states, QoSProfile(depth=1))
        self.reset_client = self.create_client(Trigger, 'sparcbot_driver/reset_encoders')

        self.drive_cmd = 0.20
        self.turn_cmd = 0.16

        self.mode = WheelCommand.MODE_COAST
        self.left_u = 0.0
        self.right_u = 0.0

        self.states = None

        self.create_timer(SEND_PERIOD_S, self.send)
        self.create_timer(PRINT_PERIOD_S, self.print_status)

    def on_states(self, msg):
        self.states = msg

    def send(self):
        msg = WheelCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = self.mode
        msg.left = self.left_u
        msg.right = self.right_u
        self.cmd_pub.publish(msg)

    def set_duty(self, left_u, right_u):
        self.mode = WheelCommand.MODE_DUTY
        self.left_u = left_u
        self.right_u = right_u
        self.send()

    def stop(self, mode):
        self.mode = mode
        self.left_u = 0.0
        self.right_u = 0.0
        self.send()

    def handle_key(self, key):
        """Return False when the user asked to quit."""
        if key in ('w', 'W'):
            self.set_duty(+self.drive_cmd, +self.drive_cmd)
        elif key in ('s', 'S'):
            self.set_duty(-self.drive_cmd, -self.drive_cmd)
        elif key in ('a', 'A'):
            self.set_duty(-self.turn_cmd, +self.turn_cmd)
        elif key in ('d', 'D'):
            self.set_duty(+self.turn_cmd, -self.turn_cmd)
        elif key == ' ':
            self.stop(WheelCommand.MODE_COAST)
        elif key in ('b', 'B'):
            self.stop(WheelCommand.MODE_BRAKE)
        elif key in ('+', '='):
            self.drive_cmd = min(MAX_CMD, self.drive_cmd + STEP)
            print(f'\nDrive command = {self.drive_cmd:.2f}')
        elif key in ('-', '_'):
            self.drive_cmd = max(MIN_CMD, self.drive_cmd - STEP)
            print(f'\nDrive command = {self.drive_cmd:.2f}')
        elif key == ']':
            self.turn_cmd = min(MAX_CMD, self.turn_cmd + STEP)
            print(f'\nTurn command = {self.turn_cmd:.2f}')
        elif key == '[':
            self.turn_cmd = max(MIN_CMD, self.turn_cmd - STEP)
            print(f'\nTurn command = {self.turn_cmd:.2f}')
        elif key in ('r', 'R'):
            if self.reset_client.service_is_ready():
                self.reset_client.call_async(Trigger.Request())
                print('\nEncoders reset.')
            else:
                print('\nsparcbot_driver/reset_encoders not available (is the driver running?)')
        elif key in ('q', 'Q'):
            return False
        return True

    def print_status(self):
        if self.states is None:
            line = 'waiting for /wheel_states (is the driver running?)'
        else:
            L, R = self.states.left, self.states.right
            line = (f'cmd L={self.left_u:5.2f} R={self.right_u:5.2f}'
                    f' | enc L={L.counts:8d} R={R.counts:8d}'
                    f' | vel L={L.velocity_m_s:7.3f} R={R.velocity_m_s:7.3f} m/s'
                    f' | drive={self.drive_cmd:.2f} turn={self.turn_cmd:.2f}')
        print('\r' + line + '      ', end='', flush=True)


def main(args=None):
    # Handle Ctrl-C ourselves so ROS is still up to send the final coast.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = PwmTeleop()
    print(HELP)

    try:
        with RawTerminal() as terminal:
            running = True
            while running and rclpy.ok():
                key = terminal.read_key()
                if key is not None:
                    running = node.handle_key(key)
                rclpy.spin_once(node, timeout_sec=0.005)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            # Send coast a few times so it is delivered before we exit.
            # (If it were lost, the driver's command timeout would coast anyway.)
            for _ in range(5):
                node.stop(WheelCommand.MODE_COAST)
                time.sleep(0.02)
            print('\n\nStopped. Both motors are coasting.')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
