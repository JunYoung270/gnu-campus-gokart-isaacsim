# Simulator-independent driver/AI command.
# steering_angle uses the common vehicle convention in radians:
# left is negative and right is positive before a platform adapter is applied.
std_msgs/Header header
float64 steering_angle
float32 throttle
float32 brake
