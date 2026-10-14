# Test fixtures

`wheel_velocity_controller_reference.csv` holds the outputs of the reference
wheel velocity controller (feedforward + PID) for 10 test sequences:
2 wheels (feedforward values from `config/sparcbot.yaml`) × 5 gain sets, 240
steps each, with a controller reset at step 120. Every row lists the inputs
(`target_m_s`, `measured_m_s`, `dt_s`, gains) and the expected outputs
(`error_m_s`, `feedforward_command`, `feedback_command`, `command`).

The tests replay the inputs through `wheel_velocity_controller.py` and check
the outputs match to 1e-9.

If you change the feedforward values in `sparcbot.yaml`, regenerate this file;
otherwise the feedforward checks will fail.
