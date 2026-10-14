# Motor command for sparcbot_driver (/wheel_cmd).

uint8 MODE_DUTY = 0          # apply left/right as normalized duty
uint8 MODE_COAST = 1         # driver outputs open, wheels spin freely (left/right ignored)
uint8 MODE_BRAKE = 2         # short brake (left/right ignored)

std_msgs/Header header
uint8 mode
float64 left                 # normalized duty in [-1, 1]; positive = robot forward
float64 right
