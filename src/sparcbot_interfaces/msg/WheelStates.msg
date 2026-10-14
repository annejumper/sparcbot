# Both wheels, published by sparcbot_driver once per control period (/wheel_states).

std_msgs/Header header
float64 dt_s                 # time since the previous sample [s]; use this as the controller dt
WheelState left
WheelState right
