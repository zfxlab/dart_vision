# CameraObservation.msg

std_msgs/Header header

# 状态
uint8 STATUS_OK=0
uint8 STATUS_NO_TARGET=1
uint8 STATUS_NOT_CALIBRATED=2
uint8 STATUS_INTERNAL_ERROR=3

uint8 status_code

# 相机光学坐标系中的单位视线：+x 向右、+y 向下、+z 向前。
# 该观测只表达方向，不提供距离或三维位置。
geometry_msgs/Vector3 bearing
