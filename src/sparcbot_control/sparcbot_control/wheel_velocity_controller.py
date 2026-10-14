"""Wheel velocity controller: feedforward + PID.
"""

from dataclasses import dataclass

from sparcbot_control.feedforward import WheelFeedforward, WheelFeedforwardConfig


@dataclass
class PIDGains:
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0


@dataclass
class WheelVelocityControllerOutput:
    error_m_s: float = 0.0            # target - measured
    feedforward_command: float = 0.0  # from WheelFeedforward
    feedback_command: float = 0.0     # from the PID
    command: float = 0.0              # what is sent to the motor


class PID:
    """Scalar PID controller."""

    def __init__(self, gains: PIDGains):
        self.gains = gains

        # TODO:

    def compute(self, error: float, dt_s: float) -> float:
        """Return the PID output for this time step.

        Args:
            error: target - measured.
            dt_s:  time since the previous call [s], > 0.
        """
        if dt_s <= 0.0:
            raise ValueError('PID.compute requires dt_s > 0')

        # TODO:
        raise NotImplementedError('TODO: implement PID.compute()')

    def reset(self) -> None:
        """Forget all history (called when the robot stops or is reset)."""
        # TODO:


class WheelVelocityController:
    """Feedforward + PID velocity controller for one wheel."""

    def __init__(self, feedforward_config: WheelFeedforwardConfig, gains: PIDGains):
        self.feedforward = WheelFeedforward(feedforward_config)
        self.pid = PID(gains)

    def compute(
        self,
        target_velocity_m_s: float,
        measured_velocity_m_s: float,
        dt_s: float,
    ) -> WheelVelocityControllerOutput:
        """Return the motor command (and its parts) for this time step."""
        # TODO:
        raise NotImplementedError('TODO: implement WheelVelocityController.compute()')

    def reset(self) -> None:
        """Reset the controller (the feedforward has no state)."""
        # TODO:
