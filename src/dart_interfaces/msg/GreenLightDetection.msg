# 消息头原样复制自输入图像；frame_id 为相机光学坐标系。
std_msgs/Header header
# 清理后的绿色掩膜中没有外轮廓，推断舱门关闭。
uint8 CLOSED=0
uint8 DETECTED=1
# 存在轮廓，但没有唯一且合格的绿灯目标，包括多个候选无法区分的情况。
uint8 NO_TARGET=2
# 图像处理失败，或检测到目标但对应 CameraInfo 不可用/射线计算失败。
uint8 ERROR=3
uint8 status 3
# 仅在 DETECTED 状态下有效，其余状态为零向量。
# 从相机光心出发的无量纲单位射线：x 右、y 下、z 前，模长为 1。
geometry_msgs/Vector3 unit_ray
# 圆拟合得分，范围为 [0,1]，不表示经过标定的概率。
float32 score
