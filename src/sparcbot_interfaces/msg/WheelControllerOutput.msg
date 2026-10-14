# One wheel's controller terms for one control step.

float64 target_m_s
float64 measured_m_s
float64 error_m_s            # target - measured
float64 feedforward_command
float64 feedback_command     # PID output
float64 command              # feedforward + feedback, before the driver's [-1, 1] clamp
