# 绿灯检测、双目测量与瞄准

## 包与接口

```text
hik_camera_driver -> dart_camera (left/right) -> dart_stereo -> dart_aiming -> dart_serial
                          image pixels          3D point      AimCommand       |
                                                                  ^            |
                                                                  + ControllerState
```

- `dart_camera`：仅检测原始全幅图像中的绿灯，发布 `GreenLightDetection`，不读取内参、不计算视线。多个候选拟合分数接近时报告 NO_TARGET。
- `dart_stereo`：接收左右检测结果与标准 `CameraInfo`，去畸变、计算视线、近似时间配对和三角测量，通过 URDF 的 TF 将两个光心和视线统一到双目中心，在该坐标系内三角测量并发布 `StereoTarget`。
- `dart_aiming`：接收三维测量和控制器消息，通过测量时刻的 TF 将目标转换到 launcher_frame，计算距离和偏转角，确认连续测量后发布 `AimCommand`。使用 C++，算法核心与 ROS 节点分开。
- `dart_serial`：串口协议字节布局保持不变，负责 ROS 字段与线协议字段的映射。

| 消息 | Topic | 时间与数据约定 |
|---|---|---|
| GreenLightDetection | /left_camera/detection、/right_camera/detection | header 原样复制图像；center_u/v、radius_px 单位 px；score 是圆拟合得分，不是概率 |
| StereoTarget | /camera/stereo_target | 左右图像时间戳中点；position 单位 m，位于 stereo_camera_center_link；ray_gap_m 是射线间距 |
| ControllerState | /controller_state | 主机接收时间；target_mode、dart_offset_rad、launcher_yaw_rad |
| AimCommand | /aim_command | 指令生成时间；yaw_error_rad 右正，distance_m 从 launcher_frame 原点测量 |

检测消息默认 ERROR=3，双目和瞄准消息默认 INVALID=2，避免默认构造被解释成关门。坐标字段只在 DETECTED / VALID 时可用。
每张输入图像产生检测消息，header 原样复制。无图像不会伪造 CLOSED。
没有收到对应图像时间戳的 CameraInfo 时不使用别帧内参猜测，等待短时间后在处理配对时报告 INVALID；驱动为图像和 CameraInfo 设置相同时间戳。

## 标定与坐标

`config/camera/cameras.yaml` 当前使用 00DA1701234 作为左相机、00DA1923281 作为右相机。实机安装必须与此一致。
两路标定 YAML 提供独立内参。两个相机的光心位置和旋转均从 dart_description 发布的 TF 获取，不再配置 right_to_left 数组。
当前 TF 的 0.30 m 基线和相机姿态来自 URDF 名义安装尺寸；实测标定结果需要更新到 TF 对应的安装变换中。
工作分辨率为 1440×1080；检测像素必须与 CameraInfo 的原始图像对应，不允许额外裁剪或缩放后直接使用原标定。

光学坐标系：x 右、y 下、z 前。双目中心 `stereo_camera_center_link`：x 前、y 左、z 上。
节点在配对图像时间戳的中间时刻查询两个光学坐标系到固定板中心的 TF，得到 (R_L, t_L) 和 (R_R, t_R)。
光心是点，位置分别为 t_L、t_R；单位视线是方向，只应用旋转：

```text
O_L = t_L                 O_R = t_R
l = R_L * bearing_left   r = R_R * bearing_right
P_L(s) = O_L + s*l        P_R(q) = O_R + q*r
P_center = (P_L(s) + P_R(q)) / 2
```

通过最小化两条射线间距求 s、q，所有求交和位置检查均在固定板中心坐标系进行。
min_depth_m 限制从各自光心沿射线前进的距离；max_distance_m 限制从固定板中心到目标的距离。
目标必须在中心前方（x > 0），射线方向本身不按中心 z 分量判断前后。
当前 URDF 给出的光心位置为左 (0.04, 0.15, 0.015) m、右 (0.04, -0.15, 0.015) m。
算法直接使用 TF，不要求两个相机完全平行或安装偏移完全对称。

当前双目配置要求左右图像时间差不超过 0.03 s、视线夹角不小于 0.05°、沿两条视线的前向深度不小于 0.5 m、目标到固定板中心的距离不超过 30 m，并且两条异面视线的最近距离不超过 0.1 m。

`dart_stereo` 的 `reference_frame` 为 `stereo_camera_center_link`，`dart_aiming` 的输出 `reference_frame` 为 `launcher_frame`。
任一相机缺少 TF 时双目发布 INVALID；CLOSED 不需要 TF。
瞄准节点按输入的 header.frame_id 查询目标测量时刻的 TF，将位置转换到发射架参考系：

```text
P_launcher = T_launcher_input * P_input
distance_m = norm(P_launcher)
yaw_error_rad = wrap_to_pi(-atan2(P_launcher.y, P_launcher.x) + dart_offset_rad)
```

位置变换同时包含旋转和平移。输出 frame_id 为 launcher_frame，距离从发射架原点量起，偏转角相对发射架 x 轴且向右为正。
有效目标缺少对应时刻 TF 时发布 INVALID，不使用最新 TF 代替历史变换；CLOSED 仍不依赖 TF 和控制器。
反馈 launcher_yaw_rad 保留原始电机角度，serial 按 motor_to_joint_sign 与 motor_zero_rad 转为 JointState。
目标已经转换到发射架坐标系，aiming 不再次减去电机反馈角。安装外参由 dart_description 的 TF 提供；其名义尺寸也需要实机核对。
`camera_system.launch.py` 直接启动驱动、检测和双目节点，默认通过 description 启动 robot_state_publisher。
若已有外部 TF 发布节点，可设置 `start_description:=false`；完整链路也支持该选项。关闭后仍需外部提供双目计算所需的 TF。

## 舱门、目标与指令状态

舱门状态按绿色轮廓推断，不使用 `LoggerPacket.door_status`。串口节点可以解析并记录该字段，但它不参与视觉状态或瞄准指令判断。
“轮廓”指 HSV / 绿色优势分割、形态学清理之后，几何和亮度筛选之前的外轮廓。它不是图像中所有物体的边缘。

### dart_camera

| status | 条件 | 解释 |
|---|---|---|
| CLOSED=0 | 清理后绿色轮廓数为 0 | 推断本侧舱门关闭 |
| DETECTED=1 | 有轮廓，且能选出明确合格绿灯 | 本侧未关闭且有目标 |
| NO_TARGET=2 | 有轮廓，但全部被筛掉或多个候选无法区分 | 本侧未关闭但无可靠目标 |
| ERROR=3 | 图像转换或处理失败 | 未知，不推断关闭 |

例如细长绿色区域或过小光点仍计为“有轮廓”，不能因其不满足圆形绿灯条件就报告 CLOSED。

### dart_stereo

先按图像时间戳配对，要求左右 frame_id 匹配配置。

| 左相机 | 右相机 | 双目状态 |
|---|---|---|
| CLOSED | CLOSED | CLOSED=0；不需要内参或三角测量 |
| DETECTED | DETECTED | 标定、视线、三角测量和参考坐标变换均通过才为 VALID=1；否则 INVALID=2 |
| 其他任何组合 | | INVALID=2 |

一侧 CLOSED、另一侧 DETECTED 或 NO_TARGET 均为 INVALID。双目无成功配对时不产生新的三维结果，由 aiming 根据最后测量时间超时失效，绝不把掉线当作 CLOSED。

### dart_aiming 与 dart_serial

| state | 条件 | 角度/距离 |
|---|---|---|
| CLOSED=0 | 新鲜双目结果为 CLOSED | 都为 0；不依赖控制器反馈和 TF |
| VALID=1 | 双目 VALID；控制器消息新鲜且模式受支持；测量时刻 TF 可用且转换后几何合法；连续帧确认通过 | 右正相对角度（包含一次飞镖补偿）、正距离 |
| INVALID=2 | 其余情况：遮挡、无目标、几何失败、等待确认、超时、TF 缺失或模式不支持 | 都为 0 |

默认要求 3 帧连续稳定测量，参数在 aiming.yaml。模式切换、补偿改变、目标关闭/无效或超时会重置确认。
旧的或重复的测量不重复计数；已收到的新测量不会被更早的 CLOSED 覆盖。
关闭证据本身也必须新鲜，最后一条 CLOSED 超时后发送 INVALID。

serial 对消息时间戳再次检查：新鲜 CLOSED 原样发送 0 并清零物理量；新鲜 VALID 且数值有限、距离为正才发送 1；其余、未知状态和指令中断均发送 2 并清零物理量。
串口节点识别 13 字节、帧头为 `0x5A` 的 ReceivePacket，以及 25 字节、帧头为 `0xD5` 的 LoggerPacket。SendPacket 仍为 12 字节、帧头为 `0xA5`，CRC 与字段顺序不变，但 state 语义已变更，MCU 必须同步为 0=CLOSED / 1=VALID / 2=INVALID。

**判据限制：** 双侧完全遮挡、绿灯熄灭或绿色分割失败而没有残留轮廓，也会被判为 CLOSED。这些情况仅靠“没有绿色轮廓”无法与真实关门区分；单侧遮挡或仍有轮廓的部分遮挡则会走 INVALID。没有额外增加舱门传感器或门体识别。

线协议 target_mode=0 仍表示前哨站，和 AimCommand.state=0 是两个独立含义。`vision_system.launch.py` 加载的 aiming 配置支持 target_mode 0—4。
五种受支持模式均指向实际测得绿灯；未实现移动目标预测、虚拟轨道目标、装甲板中心补偿。
半径和拟合分数不用于左右硬匹配（两相机焦距不同）；多灯场景仍需要进一步联合匹配。

## 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to dart_bringup --symlink-install
source install/setup.bash
# 相机感知链路
ros2 launch dart_bringup camera_system.launch.py
# 已有外部 TF 时关闭 description
ros2 launch dart_bringup camera_system.launch.py start_description:=false
# 或完整链路：相机、TF、aiming、串口
ros2 launch dart_bringup vision_system.launch.py
```

回放时可使用 `start_driver:=false`、`start_serial:=false` 和 `use_sim_time:=true`。
相机、检测器和双目配置位于 `dart_bringup/config/camera/`，瞄准配置位于 `dart_bringup/config/aiming.yaml`。完整链路默认使用 `dart_serial/config/serial.yaml`。

```bash
ros2 topic echo /camera/stereo_target
ros2 topic echo /aim_command
```

## dart_aiming C++ 结构

- include/dart_aiming/aiming_core.hpp、src/aiming_core.cpp：不依赖 ROS 的几何计算和连续帧确认。
- include/dart_aiming/aiming_node.hpp、src/aiming_node.cpp：订阅双目与控制器，执行 TF 变换，管理时效并发布指令。
- src/aiming_main.cpp：ROS 节点入口，仍使用 ros2 run dart_aiming aiming_node 启动。

参数文件仍为 dart_bringup/config/aiming.yaml。启动参数保持兼容，reference_frame 现在表示 TF 变换后的输出参考系。

## 离线残差标定界面

测量记录、拟合对比和报告导出使用独立的本地网页工具：

```bash
bash tools/residual_calibration/run.sh
```

浏览器打开 http://127.0.0.1:8501。详细数据约定、项目恢复和导出格式见 [工具说明](../tools/residual_calibration/README.md)。工具不修改实时链路；生成的补偿 YAML 需后续在 C++ 节点中接入后才能生效。
