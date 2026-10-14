"""Feedforward (given code): must match the C++ WheelFeedforward exactly."""

import math

import pytest

from reference import FEEDFORWARD, TOLERANCE, load_cases
from sparcbot_control.feedforward import WheelFeedforward, WheelFeedforwardConfig


def test_matches_reference():
    for rows in load_cases().values():
        ff = WheelFeedforward(FEEDFORWARD[rows[0]['wheel']])
        for row in rows:
            got = ff.compute(row['target_m_s'], row['measured_m_s'])
            assert math.isclose(got, row['feedforward_command'], abs_tol=TOLERANCE), row


def test_zero_target_gives_zero():
    ff = WheelFeedforward(FEEDFORWARD['left'])
    assert ff.compute(0.0, 0.0) == 0.0
    assert ff.compute(0.00005, 0.2) == 0.0


def test_breakaway_only_when_stopped():
    ff = WheelFeedforward(FEEDFORWARD['left'])
    assert ff.compute(0.01, 0.0) == pytest.approx(0.15)          # stopped -> breakaway
    assert ff.compute(0.01, 0.05) == pytest.approx(0.0701 + 1.739 * 0.01)
    assert ff.compute(-0.01, 0.0) == pytest.approx(-0.13)


def test_rejects_invalid_config():
    with pytest.raises(ValueError):
        WheelFeedforward(WheelFeedforwardConfig(forward_static=-0.1))
    with pytest.raises(ValueError):
        WheelFeedforward(WheelFeedforwardConfig(max_command=1.5))
