"""Wheel feedforward: target wheel velocity [m/s] -> normalized motor duty.

Model (from tools/dual_motor_characterization):

    forward:  u = forward_static + forward_velocity_gain * v
    reverse:  u = -(reverse_static + reverse_velocity_gain * |v|)

If the wheel is (nearly) stopped, the command is raised to at least the
breakaway duty so the wheel starts moving. The result is clamped to
[-max_command, max_command].
"""

from dataclasses import dataclass


@dataclass
class WheelFeedforwardConfig:
    forward_static: float = 0.0
    forward_velocity_gain: float = 0.0

    reverse_static: float = 0.0
    reverse_velocity_gain: float = 0.0

    forward_breakaway: float = 0.0
    reverse_breakaway: float = 0.0

    stopped_velocity_threshold: float = 0.005

    max_command: float = 1.0
    zero_velocity_threshold: float = 1e-4


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


class WheelFeedforward:
    def __init__(self, config: WheelFeedforwardConfig):
        non_negative = (
            'forward_static', 'forward_velocity_gain',
            'reverse_static', 'reverse_velocity_gain',
            'forward_breakaway', 'reverse_breakaway',
            'stopped_velocity_threshold', 'zero_velocity_threshold',
        )
        for name in non_negative:
            if getattr(config, name) < 0.0:
                raise ValueError(f'{name} must be >= 0')

        if config.max_command <= 0.0 or config.max_command > 1.0:
            raise ValueError('max_command must be in (0, 1]')

        self.config = config

    def compute(self, target_velocity_m_s: float, measured_velocity_m_s: float) -> float:
        """Return the feedforward duty in [-max_command, max_command].

        Args:
            target_velocity_m_s:   desired wheel velocity [m/s].
            measured_velocity_m_s: current wheel velocity [m/s] (used to detect a stopped wheel).
        """
        # TODO:
        raise NotImplementedError('TODO: implement WheelFeedforward.compute()')
