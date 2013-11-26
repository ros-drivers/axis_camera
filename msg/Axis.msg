uint32 SELECT_PAN=1
uint32 SELECT_TILT=2
uint32 SELECT_ZOOM=4
# Convenience
uint32 SELECT_PTZ=7
uint32 SELECT_FOCUS=8
uint32 SELECT_BRIGHTNESS=16
uint32 SELECT_IRIS=32
uint32 SELECT_AUTOFOCUS=64
uint32 SELECT_IRFILTER=128
# Selector field, default value = 0, means apply all
# Otherwise, bit-mask of above value
uint32 fields
float32 pan
float32 tilt
float32 zoom
float32 focus
float32 brightness
float32 iris
bool autofocus
bool irfilter
