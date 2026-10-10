# 时间戳为指令生成时间；frame_id 为计算瞄准结果所用的参考坐标系。
std_msgs/Header header
# 双目观测未过期，且两侧相机均报告没有绿色轮廓，推断舱门关闭。
uint8 CLOSED=0
# 目标未过期，且控制器状态和坐标变换均可用。
uint8 VALID=1
# 观测或指令未知、被遮挡、不合法或已超时。
uint8 INVALID=2
uint8 state 2
# 仅在 state=VALID 时有效，其他状态下为零。
# 相对当前发射架方向的偏转角，单位为弧度，向左为正，已加入飞镖偏角补偿。
float32 yaw_error_rad
# 二次函数拟合后的目标水平距离，单位为米。
# 非 VALID 状态下为零。
float32 distance_m
