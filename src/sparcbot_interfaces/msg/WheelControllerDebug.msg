# Published by wheel_controller every step (/wheel_controller/debug).
# Plot it live with rqt_plot / PlotJuggler, or record it with ros2 bag.

uint8 MODE_VELOCITY = 0
uint8 MODE_COAST = 1
uint8 MODE_BRAKE = 2

std_msgs/Header header
uint8 mode
float64 dt_s
WheelControllerOutput left
WheelControllerOutput right
