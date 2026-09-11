std_msgs/Header header
# 1 fixed, 2 random fixed, 3 random moving, 4 end moving; others unsupported.
uint8 target_id
# Per-dart optical-sign correction, added once by aiming.
float32 offset_rad
# Raw motor feedback; serial adapts it separately for JointState.
float32 yaw_rad
