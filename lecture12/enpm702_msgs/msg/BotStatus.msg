# The field model will take only one of the three following values
uint8 BURGER=1
uint8 WAFFLE=2
uint8 WAFFLE_PI=3


uint8 model # BURGER, WAFFLE, or WAFFLE_PI
geometry_msgs/Pose2D pose # x, y, theta
float64[2] velocities # [linear.x, angular.z]
