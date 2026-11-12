# 主机解析有效日志帧时的 ROS 时间；不是下位机采样时间。
std_msgs/Header header
# 0xD5 日志包的完整业务字段，保留下位机原始数值。
uint8 state
uint8 prepare_state
uint8 launch_station_status
uint8 is_fire_finished
uint8 fired_count_this_open
uint8 current_shot_number
uint8 current_dart_id
uint8 door_status
uint8 last_light_detected
uint8 vision_light_detected
uint8 door_session_active
uint8 autoaim_allow
uint8 door_close_inhibit_active
# 拉力单位沿用电控端 kg。
float32 string_l_force
float32 string_r_force
