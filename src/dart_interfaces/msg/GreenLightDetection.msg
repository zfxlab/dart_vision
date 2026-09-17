# 消息头原样复制自输入图像；检测坐标对应原始完整图像。
std_msgs/Header header
# 清理后的绿色掩膜中没有外轮廓，推断舱门关闭。
uint8 CLOSED=0
uint8 DETECTED=1
# 存在轮廓，但没有唯一且合格的绿灯目标，包括多个候选无法区分的情况。
uint8 NO_TARGET=2
# 图像处理失败，舱门状态未知。
uint8 ERROR=3
uint8 status 3
# 仅在 DETECTED 状态下有效；中心坐标和半径的单位均为像素。
float32 center_u
float32 center_v
float32 radius_px
# 圆拟合得分，范围为 [0,1]，不表示经过标定的概率。
float32 score
