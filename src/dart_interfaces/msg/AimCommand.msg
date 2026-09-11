std_msgs/Header header
uint8 INVALID=0
uint8 ACQUIRE=1
uint8 TRACKING=2
uint8 DEGRADED=3
# These state values form the MCU contract. ACQUIRE is orientation only.
uint8 state
# Relative to current launcher direction; right positive; dart offset added once.
float32 yaw_rad
# 3D launcher-to-light distance; modes 3/4 use the virtual light at rail q=0.28.
float32 distance_m
