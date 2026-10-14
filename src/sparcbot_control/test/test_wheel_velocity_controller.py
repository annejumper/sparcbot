"""Checks for the student's PID + feedforward controller.

Run:  colcon test --packages-select sparcbot_control && colcon test-result --verbose
"""

import math

import pytest

from reference import FEEDFORWARD, TOLERANCE, load_cases
from sparcbot_control.wheel_velocity_controller import (
    PID,
    PIDGains,
    WheelVelocityController,
)


# ---------------------------------------------------------------------------
# PID on its own
# ---------------------------------------------------------------------------

def test_pid_proportional_only():
    pid = PID(PIDGains(kp=2.0))
    assert pid.compute(0.5, 0.01) == pytest.approx(1.0)
    assert pid.compute(-0.25, 0.01) == pytest.approx(-0.5)


def test_pid_integral_accumulates_error_times_dt():
    pid = PID(PIDGains(ki=1.0))
    assert pid.compute(1.0, 0.1) == pytest.approx(0.1)
    assert pid.compute(1.0, 0.1) == pytest.approx(0.2)
    assert pid.compute(-2.0, 0.05) == pytest.approx(0.1)


def test_pid_derivative_is_zero_on_first_call():
    pid = PID(PIDGains(kd=1.0))
    assert pid.compute(1.0, 0.01) == pytest.approx(0.0)
    assert pid.compute(1.5, 0.01) == pytest.approx(50.0)


def test_pid_reset_clears_history():
    pid = PID(PIDGains(kp=1.0, ki=1.0, kd=1.0))
    pid.compute(1.0, 0.1)
    pid.compute(2.0, 0.1)
    pid.reset()
    # After reset: no integral history, and no derivative on the first call.
    assert pid.compute(1.0, 0.1) == pytest.approx(1.0 + 0.1)


def test_pid_rejects_non_positive_dt():
    pid = PID(PIDGains(kp=1.0))
    with pytest.raises(ValueError):
        pid.compute(1.0, 0.0)


# ---------------------------------------------------------------------------
# Full controller vs. the reference implementation
# ---------------------------------------------------------------------------

@pytest.mark.parametrize('case_id,rows', sorted(load_cases().items()))
def test_controller_matches_reference(case_id, rows):
    first = rows[0]
    controller = WheelVelocityController(
        FEEDFORWARD[first['wheel']],
        PIDGains(kp=first['kp'], ki=first['ki'], kd=first['kd']),
    )

    for row in rows:
        if row['reset_before']:
            controller.reset()

        out = controller.compute(row['target_m_s'], row['measured_m_s'], row['dt_s'])

        where = f"case {case_id} ({first['wheel']}), step {row['step']}"
        for field in ('error_m_s', 'feedforward_command', 'feedback_command', 'command'):
            expected = row[field]
            got = getattr(out, field)
            assert math.isclose(got, expected, abs_tol=TOLERANCE), (
                f'{where}: {field} = {got}, expected {expected}')
