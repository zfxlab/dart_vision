# CameraObservation.msg

std_msgs/Header header

# 目标已通过筛选，且单位视线计算成功。
uint8 STATUS_OK=0
# 没有通过全部筛选条件的目标；候选轮廓情况由节点日志提供。
uint8 STATUS_NO_CANDIDATE=1
# 经过图像清理后没有轮廓
uint8 STATUS_NO_CONTOUR=2
# 图像处理、标定匹配或单位视线计算失败；具体原因由节点日志提供。
uint8 STATUS_ERROR=3

uint8 status_code

# 相机光学坐标系中的单位视线：+x 向右、+y 向下、+z 向前。
# 该观测只表达方向，不提供距离或三维位置。
geometry_msgs/Vector3 bearing

uint8 STATUS_AMBIGUOUS=4
uint8 STATUS_ACQUIRING=5
uint8 STATUS_TIMEOUT=6
float32 quality
uint32 candidate_count
float32 center_u
float32 center_v
float32 radius_px
