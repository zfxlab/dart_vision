# 测量时间为配对的左右图像时间戳的中间时刻。
std_msgs/Header header
# 配对的左右相机检测结果均明确报告 CLOSED。
uint8 CLOSED=0
uint8 VALID=1
# 其他检测结果组合，或标定参数不可用、三角测量失败。
uint8 INVALID=2
uint8 status 2
# 仅在 VALID 状态下有效；位置位于 header.frame_id 指定的坐标系中，单位为米。
# x/y 为视线的 XY 投影交点，z 为两条视线在交点处的平均高度。
geometry_msgs/Point position
# header.frame_id 原点到目标的XY 平面水平距离，单位为米；仅 VALID 时有效，其他状态为零。
float64 distance
# 相对 header.frame_id 的 +x 轴的水平偏转角，单位为弧度，向右为正。
# 坐标系约定 x 前、y 左、z 上；yaw = -atan2(position.y, position.x)。
# 不包含飞镖偏角补偿；仅 VALID 时有效，其他状态为零。
float64 yaw
# 两条空间视线在水平交点处的高度差绝对值，单位为米；仅 VALID 时有效。
float32 height_gap_m
