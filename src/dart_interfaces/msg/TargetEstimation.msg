# TargetEstimate.msg

std_msgs/Header header

# 目标检测状态
uint8 STATUS_OK=0
uint8 STATUS_NO_TARGET=1
uint8 STATUS_UNKOWN=2
uint8 STATUS_DOOR_CLOSE=3

uint8 target_state

# 基地中心偏转绝对角度
float32 base_yaw_rad

# 绿灯中心偏转像素角
float32 light_yaw_rad

# 装甲板中心偏转角
float32 armor_yaw_rad

# 飞镖释放位置与装甲板中心距离
float32 distance_m
