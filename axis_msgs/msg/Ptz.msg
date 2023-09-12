# The pan position or velocity of the camera, in radians or rad/s
# 0.0 is forward / stopped
# + is to the left (anticlockwise)
# - is to the right (clockwise)
# Valid ranges:
#  position: -pi to pi
#  speed: -100 to 100
float32 pan

# The tilt position or velocity of the camera, in radians or rad/s
# 0.0 is parallel with the camera's base / stopped
# + is upward
# - is downward
# Valid ranges:
#  position: -pi/2 to pi/2
#  speed: -100 to 100
float32 tilt

# The camera's zoom level or speed
# The allowed range varies depending on whether we're using position or velocity control
# Valid ranges:
#  position: 0 to 9999
#  speed: -100 to 100
float32 zoom
