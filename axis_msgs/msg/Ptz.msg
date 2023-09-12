# The pan position or velocity of the camera, in radians or rad/s
# 0.0 is forward / stopped
# + is to the left (anticlockwise)
# - is to the right (clockwise)
float32 pan

# The tilt position or velocity of the camera, in radians or rad/s
# 0.0 is parallel with the camera's base / stopped
# + is upward
# - is downward
float32 tilt

# The camera's zoom level or speed
# The allowed range varies depending on whether we're using position or velocity control
# Position control range: 0 to 9999
# Velocity control range: -100 to +100
float32 zoom
