# Overall result header. Its stamp is the newest accumulated LiDAR frame and
# frame_id is output_frame.
std_msgs/Header header

uint8 STATUS_OK=0
uint8 STATUS_INPUT_INVALID=1
uint8 STATUS_TF_UNAVAILABLE=2
uint8 STATUS_MODEL_UNAVAILABLE=3
uint8 STATUS_INSUFFICIENT_POINTS=4
uint8 STATUS_BASE_UNRELIABLE=5
uint8 STATUS_BASE_UNSTABLE=6
uint8 STATUS_ARMOR_UNRELIABLE=7
uint8 STATUS_ARMOR_UNSTABLE=8
uint8 STATUS_OUTPUT_TF_UNAVAILABLE=9

uint8 BASE_UNKNOWN=0
uint8 BASE_OPEN=1
uint8 BASE_CLOSED=2

# valid is true only when the complete base + armor + green-light result has
# passed geometric/temporal checks and t_output_lidar is available.
bool valid
float32 confidence
uint8 status_code
string status_message

bool base_valid
uint8 base_state
float32 base_confidence
float32 base_state_score_margin
uint32 base_consecutive_frames
# Pose frame is authoritative in base_pose.header. It is output_frame when that
# TF is available and falls back to lidar_frame for an otherwise valid estimate.
geometry_msgs/PoseStamped base_pose
float32 base_truncated_residual_m
float32 base_inlier_rate
float32 base_coverage

bool armor_valid
float32 armor_confidence
uint32 armor_consecutive_frames
# Pose frame is authoritative in armor_pose.header, with the same output/lidar
# fallback rule as base_pose.
geometry_msgs/PoseStamped armor_pose
float64 armor_axis_position_m
float32 armor_truncated_residual_m
float32 armor_inlier_rate
float32 armor_coverage

# green_light_lidar is expressed in lidar_frame; green_light_output is
# expressed in output_frame. The distance is from the lidar_frame origin.
geometry_msgs/PointStamped green_light_lidar
geometry_msgs/PointStamped green_light_output
float64 green_light_distance_m

uint32 accumulated_frame_count
uint32 accumulated_point_count
