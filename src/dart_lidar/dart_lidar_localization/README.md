# dart_lidar_localization

实际运行使用description/site中的固定基地位姿，只定位移动检测模块。基地NDT/GICP只在独立标定工具中执行。

## 目录

- `src/io/`：共用PCD/PLY模型加载与package路径解析。
- `src/module/`：全滑轨粗细搜索、质量与多解判断。
- `src/calibration/`：基地NDT/GICP、质量验收、诊断文件写入。
- `include/.../calibration/confirmation_gate.hpp`：自动导出前的多窗口确认。
- `src/ros/module_localization_*`：实际运行入口。
- `src/ros/base_calibration_*`：标定工具入口。
- `test/`：固定变换、模块搜索、歧义、标定确认和文件内容回归测试。

## 实际运行

```bash
ros2 launch dart_bringup lidar_system.launch.py
```

公共入口支持 `ros2 launch dart_bringup lidar.launch.py mode:=module`；
选择 `mode:=calibrate` 时启动基地标定，默认 `module`。
两种模式共用驱动、预处理与累积；`start_localization:=false` 可关闭所选定位/标定节点。
公共入口不启动description或串口，应单独提供TF与yaw；完整系统入口也支持同一mode参数。

共用20Hz Livox预处理与滑动累积。模块节点只读取模块模型，
缓存固定的 `base_nominal_link <- rail_origin_link` TF，
每个累积窗口转换到滑轨坐标系后从头搜索。模型加载失败报启动错误；
TF暂缺、输入异常或搜索失败均输出不可用，不用默认位置冒充测量。

`module_localization.yaml` 只保留模型路径和候选显示开关。
算法默认值集中在 `ModuleLocalizationParameters`：
滑轨+x、范围[0, 0.56]m，粗步长5mm，细步长0.5mm，
细搜索半窗2cm，匹配距离2.5cm，RMSE上限1.5cm，
模板覆盖比例下限0.30，ROI至少10个实测点、匹配至少10个模板点。
相距至少3cm的候选若评分差不足1mm则拒绝为多解。
滑轨范围与当前description一致；更换机构时应同步修改该默认值。

`/lidar/observation` 为Reliable消息：
```text
std_msgs/Header header  # frame_id=rail_origin_link，stamp=累积窗口最新帧时间
bool available
float64 position_m     # 沿滑轨+x的位移m；不可用时NaN
```

不再包含基地位姿或基地状态。无新数据不重复发布；下游必须检查时间是否过期，
并注意观测来自整段窗口而非瞬时采样。雷达不发布基地TF或模块joint。
当前target_estimation尚无雷达消息消费者，本次未替它添加融合或joint发布逻辑。

`/lidar/module/candidate_model` 为Best Effort点云，将候选模块模板放置在
`base_nominal_link` 中；候选可能被质量/歧义检查拒绝，不能据此视为有效观测。

## 基地标定

```bash
ros2 launch dart_bringup lidar.launch.py mode:=calibrate
```

该入口启动雷达预处理/累积和 `base_calibration_node`，
不启动模块运行节点、不发布 `/lidar/observation`、不修改site。

`base_calibration.yaml` 默认只用GICP；初值偏差较大可将 `coarse_enabled` 设为true启用NDT。
每轮从本次启动缓存的site TF开始；默认输入点数500、有效对应350的标定门槛与实际模块运行无关。
其余分辨率、迭代次数及质量门槛集中在标定节点构造函数。
2秒预算在各阶段完成后检查，不是硬实时中断。

调试点云均为Best Effort，坐标系 `base_link`：

- `/lidar/calibration/candidate_aligned`：完整候选，包括质量拒绝结果。
- `/lidar/calibration/aligned`：本轮通过基地质量验收的结果。

自动导出需要三次通过质量验收的结果，
与首个确认结果位置差不超过1cm、旋转差不超过0.005rad，每次计数至少相隔0.6秒。
失败、位姿跳变或时间倒退会重置确认。导出最后一个通过确认的真实候选，不另造平均位姿。
`output.save_once=true` 时每个进程最多自动写一次，bag循环不重置此限制。
输出目录默认 `results/lidar`，不再每帧在site目录生成文件。
如果扩大累积窗口超过0.6秒，应相应调整确认间隔常量，避免高度重复窗口计数。

始终无法验收时，可主动导出最新完整候选：

```bash
ros2 service call /lidar/calibration/export_candidate std_srvs/srv/Trigger '{}'
```

手动导出允许QualityRejected，并明确记录 `base.available=false`、
`confirmed=false`及失败原因。最新输入没有完整候选时服务返回失败。
反复主动调用可导出多份文件，不影响自动一次导出。

人工检查导出文件的 `base.six_dof`，将其直接替换site中当前red/blue的
`xyz_m/rpy_rad`（不是在旧值上再累加），然后重启description与雷达节点。
运行时初值仍只从description TF获得，不重复读取site。

## 回放

有配套yaw历史的bag：

```bash
ros2 launch dart_bringup lidar_system.launch.py start_driver:=false start_serial:=false use_sim_time:=true
ros2 bag play <bag> --clock
```

标定时将入口替换为 `lidar_calibration.launch.py`。
只有点云的旧bag可在固定yaw假设下用GUI提供joint，但不能恢复录制时真实转动。
使用系统时间与固定yaw调试时不要启用use_sim_time；
使用仿真时间时点云处理、yaw发布者和robot_state_publisher必须使用同一时钟。
