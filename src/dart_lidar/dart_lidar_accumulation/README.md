# dart_lidar_accumulation

连续接收已经在预处理阶段完成 yaw 运动补偿的 `sensor_msgs/PointCloud2`，按消息时间戳维护有限长度的滑动窗口。首次达到最小帧数时立即发布，之后按照 `publish.rate_hz` 和 `publish.min_new_frames` 控制输出；发布后不会清空窗口。

输入必须已经位于 `accumulation_frame`，本包不再读取 yaw、不判断电机是否停止，也不在窗口内冻结 TF。
实时数据和 rosbag 回放使用同一逻辑；回放时应使用 ROS 仿真时间并保持源时间戳单调。

发布节拍由输入点云时间戳驱动，而不是墙上时钟定时器，因此 rosbag 加速、减速或暂停不会重复发布旧点云。窗口中的帧会在超过 `accumulation.window_duration_s`、`accumulation.max_frames` 或 `accumulation.max_points` 时从最旧帧开始淘汰。
