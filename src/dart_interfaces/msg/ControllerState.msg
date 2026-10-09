# 时间戳为主机收到控制器数据包的时间。
std_msgs/Header header
# 沿用下位机的模式编号：0 表示前哨站，不表示禁用。
uint8 TARGET_OUTPOST=0
uint8 TARGET_FIXED=1
uint8 TARGET_RANDOM_FIXED=2
uint8 TARGET_RANDOM_MOVING=3
uint8 TARGET_END_MOVING=4
uint8 target_mode
# 飞镖偏角补偿，单位为弧度，向左为正；由 dart_aiming 叠加一次。
float32 dart_offset_rad
# 电机反馈角，单位为弧度，向左为正；串口节点校零后发布 JointState。
float32 launcher_yaw_rad
