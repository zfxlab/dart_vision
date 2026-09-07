# dart_lidar_accumulation

该包只接收标准 `sensor_msgs/PointCloud2`，不依赖 Livox 私有消息。

- `triggered_online`：由 `ControllerState.yaw_rad` 自动判稳，等待跨包保护时间后冻结最新 TF；使用
  `steady_clock` 控制 1s 截止时间，完成一次测量后等待 yaw 再次移动。
- `bag_offline`：不订阅电控，按消息自身时间戳形成连续固定窗口。播放 rosbag 时改变速率或暂停不会改变
  窗口覆盖的数据范围。它适合已按 yaw 静止区间裁剪过的 bag，不应跨越 yaw 运动区间累积。

两种模式都只在每个窗口开始时查询一次 TF，窗口内使用同一个冻结变换。输出在线模式使用发布时 ROS
时间；离线模式保留窗口最后一帧的源时间。后续配准应把每次输出视作不可变测量快照。

`mode`、话题和 `accumulation_frame` 只在启动时加载。其余算法参数支持运行时原子更新：空闲时立即
生效；正在测量时暂存，并在下一个累积窗口开始前生效，从而保证单个输出窗口只使用一套配置。
