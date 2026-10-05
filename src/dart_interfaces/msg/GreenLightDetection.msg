# 消息头原样复制自输入图像；frame_id 为相机光学坐标系。
std_msgs/Header header
# 清理后的绿色掩膜中没有外轮廓，推断舱门关闭。
uint8 CLOSED=0
uint8 DETECTED=1
# 存在轮廓，但没有唯一且合格的绿灯目标，包括多个候选无法区分的情况。
uint8 NO_TARGET=2
# 图像转换、目标检测或像素观测验证失败。
uint8 ERROR=3
uint8 status 3
# 仅在 DETECTED 状态下有效，其余状态均为零。
# 像素坐标原点在图像左上角，x 向右、y 向下。
float32 center_x_px
float32 center_y_px
# 目标轮廓的最小外接圆半径。
float32 radius_px
