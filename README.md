# Dart vision

RoboMaster 飞镖镖架感知与瞄准系统。MID-70 固定在镖架主体，相机随 yaw 转动。相机和雷达独立观测，估计层统一几何，决策层生成 `packet.hpp` 定义的串口输出。

## 运行

目标运行环境为 Ubuntu 24.04 + ROS 2 Jazzy + Python 3.12（包含 rclpy、tf2_ros、sensor_msgs_py）、NumPy/SciPy。相机检测、相机驱动和串口为 C++；新增雷达、估计和决策节点为 Python，点云计算使用 NumPy/SciPy。海康 MVS 和 Livox SDK 依照各驱动说明安装。

在本仓库根目录：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src third_party tools --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
source install/setup.bash
ros2 launch dart_bringup system.launch.py
```

依赖安装前确认 ROS 2 环境已加载，三个子模块已经拉取。海康 SDK 等厂商依赖不保证由 rosdep 自动安装。

### 从 Ubuntu 22.04 / Humble 迁移

不要复用旧 `build/`、`install/` 和 Python 3.10 虚拟环境；先备份，再在只加载 Jazzy 的终端中重新构建。不要加载旧的 `install/setup.bash`。

可选的 Python 开发环境（先安装 uv；ROS 系统依赖仍通过 rosdep 安装）：

```bash
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv pip compile requirements-uv.in --python .venv/bin/python -o requirements-uv.lock
uv pip install --python .venv/bin/python -r requirements-uv.lock
source .venv/bin/activate
```

`--system-site-packages` 用于访问系统安装的 ROS Python 依赖。使用虚拟环境版本的 NumPy/SciPy 运行节点时，启动前也需要激活该环境。上面的构建命令固定使用系统 Python 3.12 生成 ROS 接口。

Livox SDK vendor 保留 MID-70 使用的 SDK 2.3.1，并仅对 GCC 13 及以上的外部 SDK 构建放宽 `c++20-compat` 警告的错误处理；其余 `-Werror` 检查保留。首次构建需联网下载 SDK。

录包回放：

```bash
ros2 launch dart_bringup system.launch.py start_drivers:=false start_serial:=false use_sim_time:=true lidar_timestamp_source:=header
ros2 bag play YOUR_BAG --clock
```

录包需包含点云、图像、CameraInfo、controller_state 与 joint_states，时间必须处于同一 ROS 时钟域。不要同时回放旧 target_estimation、aim_command、observation_window 或基地参考，否则会与在线节点重复发布。

## 包与话题

| 包 | 输入 | 输出 |
|---|---|---|
| dart_serial | 原始接收包、aim_command | controller_state、joint_states、原始发送包 |
| dart_camera | camera/image_raw、camera/camera_info | camera/observation |
| dart_lidar | livox/lidar（PointCloud2）、observation_window、固定 TF | lidar/visibility、lidar/base_reference、lidar/observation |
| dart_target_estimation | 相机/雷达观测与 TF | observation_window、target_estimation |
| dart_aiming | target_estimation、controller_state、当前 TF | aim_command |
| dart_description | 启动先验、joint_states | TF |

基地参考和观察窗口使用 transient-local QoS；相机、雷达和电控观测使用 sensor-data QoS。`dart_interfaces` 是内部消息的唯一来源。

## 命令语义

发送协议完全保留 `src/dart_serial/include/dart_serial/packet.hpp` 的 12 字节布局：`header/state/yaw_rad/distance_m/crc`。该文件仍有旧注释，当前业务定义以本节和 AimCommand.msg 为准。

| target_id | yaw 目标 | distance 终点 |
|---|---|---|
| 1 固定 | 实际绿灯中心 | 实际绿灯中心 |
| 2 随机固定 | 实际绿灯中心 | 实际绿灯中心 |
| 3 随机移动 | 基地指定中心 | 模块位于滑轨 q=0.28 m 时的虚拟绿灯中心 |
| 4 末端移动 | 基地指定中心 | 同上 |

不支持 target_id=0，其余未定义值也输出 INVALID。target_id 就是本系统使用的比赛模式，不推断额外比赛阶段或计时。

所有距离从 launcher_frame 原点起算，为三维欧氏距离。发送 yaw 为相对当前发射朝向的角度，右正、左负，单位 rad：

`yaw_send = wrap(atan2(-target_in_launcher.y, target_in_launcher.x) + offset_rad)`

相机光学 bearing 必须先通过相机外参转换。URDF 中相机的 -2° 安装角会被包含，不与每发飞镖的 offset 混用。offset 仅由 dart_aiming 加一次。串口反馈角保持原值发布到 ControllerState，JointState 使用 `motor_to_joint_sign * (raw_yaw - motor_zero_rad)`。

新增 state 编码须由电控按以下定义消费，包布局不变：

| state | 含义 |
|---|---|
| 0 INVALID | 无可用解；yaw 和 distance 均置零 |
| 1 ACQUIRE | 前两种模式丢失实际绿灯，用基地中点参考提供粗略转向；不是有效实际绿灯解 |
| 2 TRACKING | 使用实测基地参考，当前所需目标数据有效 |
| 3 DEGRADED | 使用启动基地先验，当前所需目标数据有效 |

这些状态表示瞄准解质量，不等同于舱门机械全开或裁判系统允许发射。传感器只能判断观察通道，控制端仍需理解该区别。命令超时后串口节点持续补发 INVALID；整台视觉计算机离线时的通信超时由电控处理。

## 舱门与每次开门初始化

雷达先在 livox_frame 的近场 ROI 检查门板遮挡，再执行远场处理。稳定且有空间范围的近场回波产生 BLOCKED；近场没有回波且目标方向有足够远场回波产生 CLEAR；没有回波、少量不明确回波或传感器超时产生 UNKNOWN。没检测到绿灯不作为关门依据。

估计层对证据进行时间防抖，管理开门 epoch：

1. 启动时确认已开放，或确认关闭后再次开放，创建新 epoch。
2. UNKNOWN 后重新 CLEAR 不重复创建 epoch。
3. 雷达收到新 epoch，清空积累、确认计数和旧模块结果，基地参考恢复启动先验。
4. 积累本次点云，进行基地平移配准；独立积累窗口中的结果一致才接受。
5. 通过后锁定本周期实测基地参考，并增加 reference_revision。
6. 超时未通过，本周期锁定启动先验。
7. 模块持续识别，所有模式均执行；关闭或状态不明确时不输出有效模块解。

base_link 是启动先验，运行时不覆盖静态 TF。BaseReference.pose 是 fixed_frame 下的完整基地参考，但本版在线配准只调整平移，姿态沿用先验。估计层使用相同 epoch/revision 的观测；跨周期、旧版本和过期结果不参与融合。

## 感知算法

相机保留 HSV/绿色优势、形态学、圆形几何和亮度对比筛选，输出加权中心及去畸变单位射线。增加多候选歧义拒绝、时间关联、连续帧确认和图像超时。支持 plumb_bob 标定，其他模型明确拒绝。换镜头后需要重新标定。

雷达使用独立的基地积累和模块短窗口积累。容量不足时在各帧降采样，保留初始化所需时间跨度。基地使用现有 PCD 的平移约束 trimmed ICP，限制偏移、残差、支持率和空间覆盖；不拟合无观测约束的完整姿态。该方法是待实测验证的基线，无法保证未知护甲姿态或大面积遮挡下仍配准成功，此时按质量门限回退先验。

模块使用 STL 表面采样模型。先去除与已知基地表面重合的背景，再沿滑轨完整搜索 0～0.56 m，检查支持率、残差及相隔位置的歧义。长期积累引起的多位置混合应被拒绝，不能作为当前模块位置输出。

实际绿灯优先由雷达模块位移及 URDF 偏移恢复；雷达无有效解时，用图像时刻的完整相机射线与绿灯运动直线求最近点，检查正深度、轨道边界、交会残差和近似平行退化。两者同时有效但不一致时，前两种模式不输出有效瞄准解。后两种模式不依赖实际模块观测。

## 参数与时间

最终参数集中在 dart_bringup/config：

- system.yaml：初始化、门 ROI、模块匹配、数据超时、基地中心点。
- site/default.yaml：启动基地先验，保留已有现场配置。
- serial.yaml：串口及电机角到 URDF 关节角的转换。
- camera/：相机驱动、标定及绿灯检测参数。
- lidar/：MID-70 驱动配置，输出格式改为 PointCloud2。

**上机前必须落实的几何参数**：门板 ROI 是否处于 MID-70 可观测范围、门板回波覆盖、base_aim_point 的实际定义、电机角正方向和零位。当前 near_lower/near_upper 及 base_aim_point 是显式默认值，未经过现场数据标定。开门确认时间只是传感器防抖，不能用来代替机械全开时间。

MID-70 默认以 ROS 接收时间给处理结果标时（timestamp_source=receive），避免直接把未同步设备时钟当 ROS 时间。已验证驱动时钟同步或回放时可切换 header。相机驱动改为使用 SDK 返回后的主机接收时刻，扣除像素转换/发布延迟；这仍不是曝光时间。两路接收时间的系统性延迟需要实测，当前实现没有宣称硬件同步。

Python 节点采用单线程回调；点数上限限制计算量，但硬件上的处理频率和积压情况需要录包计时验证。参数修改后重启系统，避免在一次积累过程中混用参数。

## 验证

无 ROS 环境下的测试（需要 numpy、scipy）：

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
g++ -std=c++17 -Isrc/dart_serial/include tests/test_packet.cpp src/dart_serial/src/packet.cpp src/dart_serial/src/packet_parser.cpp src/dart_serial/src/crc.cpp -o /tmp/dart_packet_test
/tmp/dart_packet_test
```

ROS 环境中另执行：

```bash
colcon test --packages-select dart_lidar dart_aiming
colcon test-result --verbose
```

测试包含真实模型单位、基地配准、模块位移、积累时间跨度、光学符号、offset、射线退化、开关状态防抖、模式距离和数据过期。test_pipeline.py 运行实际 Python 节点回调，但使用本地 ROS/TF 替身；它验证消息流和生命周期，不验证 DDS、生成消息 ABI、TF 缓存时序或 ROS 2 二进制兼容性。

Ubuntu 24.04 / Jazzy 迁移验证：13 个包全量编译通过；25 项 Python 用例、`dart_lidar` / `dart_aiming` 的 colcon 测试及串口协议测试通过；Python 3.12 消息类型支持库和业务节点导入、`system.launch.py --show-args` 解析通过。启动参数解析不代表系统已实际启动。

本次验证机器的 rosdep 检查仍缺少以下系统依赖，完整运行前需要安装：

```bash
sudo apt install ros-jazzy-image-transport-plugins ros-jazzy-xacro libapr1-dev libaprutil1-dev
```

真实相机/雷达回放和电控联调仍需在设备上验证，不能把本机编译和测试通过理解为已具备实机精度。
