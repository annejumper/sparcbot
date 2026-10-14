"""Helpers for replaying the reference fixture."""

import csv
from collections import defaultdict
from pathlib import Path

from sparcbot_control.feedforward import WheelFeedforwardConfig

FIXTURE = Path(__file__).parent / 'fixtures' / 'wheel_velocity_controller_reference.csv'

# Must match wheel_controller.<wheel>.feedforward in sparcbot_bringup/config/sparcbot.yaml
FEEDFORWARD = {
    'left': WheelFeedforwardConfig(
        forward_static=0.0701, forward_velocity_gain=1.739,
        reverse_static=0.062, reverse_velocity_gain=1.7162,
        forward_breakaway=0.15, reverse_breakaway=0.13,
        stopped_velocity_threshold=0.005, max_command=1.0,
        zero_velocity_threshold=0.0001),
    'right': WheelFeedforwardConfig(
        forward_static=0.033, forward_velocity_gain=1.639,
        reverse_static=0.031, reverse_velocity_gain=1.700,
        forward_breakaway=0.10, reverse_breakaway=0.13,
        stopped_velocity_threshold=0.005, max_command=1.0,
        zero_velocity_threshold=0.0001),
}

TOLERANCE = 1e-9


def load_cases():
    """Return {case_id: [row, ...]} with numeric fields converted to float."""
    cases = defaultdict(list)
    with FIXTURE.open() as f:
        for row in csv.DictReader(f):
            for key, value in row.items():
                if key not in ('case', 'wheel', 'step', 'reset_before'):
                    row[key] = float(value)
            row['step'] = int(row['step'])
            row['reset_before'] = row['reset_before'] == '1'
            cases[int(row['case'])].append(row)
    return dict(cases)
