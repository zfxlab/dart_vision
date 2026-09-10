# 移动检测模块的一维滑轨位置观测。
# stamp为累积窗口最新一帧的时间，不代表无延迟瞬时测量。
# frame_id固定为rail_origin_link。
std_msgs/Header header

# 本轮搜索通过验收才为true；无新输入不重复发布，下游须检查时间是否过期。
bool available

# 相对滑轨零位、沿+x方向的位移，单位m；available=false时为NaN。
float64 position_m
