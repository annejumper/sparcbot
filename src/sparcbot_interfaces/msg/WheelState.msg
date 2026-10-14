# State of one wheel, as estimated by sparcbot_driver from its encoder.

int64 counts                 # encoder counts since the last reset
float64 position_rad         # wheel angle since the last reset
float64 velocity_raw_rad_s   # finite-difference velocity (unfiltered)
float64 velocity_rad_s       # low-pass filtered velocity
float64 velocity_m_s         # filtered linear velocity of the wheel rim (radius * velocity_rad_s)
float64 command              # normalized duty currently applied to the motor, in [-1, 1]
