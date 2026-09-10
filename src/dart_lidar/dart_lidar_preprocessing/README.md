# dart_lidar_preprocessing

将 Livox `CustomMsg` 转换为 `base_nominal_link` 中的运动补偿点云。节点先完成逐点时间补偿，再执行距离过滤和体素降采样，输出 `lidar/preprocessed`。

## 时间处理

Livox `header.stamp/timebase` 可能来自雷达设备运行时钟，不能直接与电控 ROS 时间混用。因此本节点只使用 `CustomPoint.offset_time` 表示包内相对时间，并将收到消息时的 ROS 时间视为最后一个点的时间：

```text
t_point = t_receive_ros - max(offset_time) + offset_time
t_yaw   = t_point + yaw_query_offset_s
```

节点按 `time_bin_s` 对点分组，并查询每组时刻的历史 TF `base_nominal_link <- livox_frame`。该 TF 通过 `/joint_states` 中的 `launcher_yaw_joint` 和 robot_state_publisher 得到，所以转动期间每组点都会使用对应 yaw，而不是整包使用同一角度。

`yaw_query_offset_s` 是雷达数据链与电控 yaw 数据链之间的常数相对延迟：

- 正值：为雷达点使用更晚的 yaw。
- 负值：为雷达点使用更早的 yaw。
- 初始保持 `0.0`，通过转动场景下的重影最小化实验标定。

如果目标时刻的 TF 尚未来到，消息会进入短队列；超过 `tf_wait_timeout_s` 后整包丢弃，避免使用错误角度污染后续累积。

## rosbag 回放

回放时必须启用 `use_sim_time:=true`、播放 `/clock`，并同时播放与点云配套的 `/joint_states`。如果旧包只录制了 `/livox/lidar`，就没有历史 yaw，无法完成转动运动补偿；只能另外发布匹配的 yaw/TF，或关闭预处理节点改用已经补偿好的点云。

默认参数位于 `dart_bringup/config/lidar/preprocessing.yaml`。
