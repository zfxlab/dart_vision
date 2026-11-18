# Dart Vision

基于 ROS 2 Jazzy 的飞镖视觉与瞄准系统。项目从双海康工业相机获取图像，检测绿色目标，
通过双目几何计算目标位置，再结合控制器状态和 TF 生成瞄准指令；同时提供串口通信和
Livox 雷达调试入口。

```text
hik_camera_driver
        │ Image + CameraInfo
        ▼
dart_camera（上下相机绿灯检测）
        │ GreenLightDetection
        ▼
dart_stereo（双目三角测量）
        │ StereoTarget
        ▼
dart_aiming（坐标变换、平滑与确认）
        │ AimCommand
        ▼
dart_serial ⇄ 控制器
```

## 环境

- Ubuntu 24.04
- ROS 2 Jazzy
- C++17
- OpenCV
- 支持 x86_64 和 aarch64 的 Hikrobot MVS 运行库已随相机驱动子模块提供

安装仓库管理、构建和代码检查所需的基础工具：

```bash
sudo apt update
sudo apt install git-lfs clang-format clang-tidy \
  python3-colcon-common-extensions python3-rosdep pipx \
  ros-jazzy-rosbag2 ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-image-transport-plugins
pipx install uv
```

`clang-format` 使用仓库根目录的 `.clang-format`，`clang-tidy` 使用 `.clang-tidy`；`uv`
用于创建隔离的 Python 工具环境并安装 Ruff，不参与 ROS 节点运行。

首次安装 `rosdep` 后需要进行一次系统级初始化，之后普通用户更新依赖索引即可：

```bash
# 每台机器只需执行一次；已经初始化时跳过此命令
sudo rosdep init
rosdep update
```

仓库使用 Git LFS 管理模型、点云和 PDF 文档等大文件，并使用 Git 子模块管理相机驱动、机器人描述和
Livox 驱动。首次拉取后执行：

```bash
git lfs install
git lfs pull
git submodule update --init --recursive
git submodule foreach --recursive git lfs pull
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src third_party --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

相机驱动默认从仓库内的 MVS SDK 查找头文件和当前架构的运行库，不需要单独安装
Hikrobot MVS。如需使用系统 SDK，可在构建时设置 `HIK_MVS_ROOT`。

### 设备权限

串口默认使用 `/dev/ttyACM0`。将当前用户加入 `dialout` 用户组后，需要注销并重新登录，
新会话才会获得设备访问权限：

```bash
sudo usermod -aG dialout "$USER"

# 重新登录后检查用户组和设备权限
groups
ls -l /dev/ttyACM0
```

不要使用 `chmod 777` 作为长期方案：设备重新插拔后权限会恢复。若设备节点不属于
`dialout`，应根据实际 USB 设备添加精确的 udev 规则。Hikrobot 工业相机还需要厂商提供的
udev 规则；出现 `device access denied` 时，应先确认 MVS 客户端能够取流并检查相机的 udev
配置，同时确保没有其他程序以独占方式打开相机。

## 运行

启动完整视觉、瞄准和串口链路：

```bash
ros2 launch dart_bringup vision_system.launch.py
```

仅启动机器人描述、双相机、绿灯检测和双目测量：

```bash
ros2 launch dart_bringup camera_system.launch.py
```

回放 rosbag 或接入外部图像时可以关闭相机驱动；已有外部 TF 发布器时可以关闭机器人描述：

```bash
ros2 launch dart_bringup vision_system.launch.py \
  start_driver:=false \
  start_description:=false \
  start_serial:=false \
  use_sim_time:=true
```

常用独立入口：

```bash
# 单个绿灯检测节点，namespace 和参数文件均可覆盖
ros2 launch dart_bringup green_light_detector.launch.py namespace:=camera

# 串口节点
ros2 launch dart_bringup serial.launch.py

# Livox 驱动和静态 TF
ros2 launch dart_bringup lidar_debug.launch.py
```

使用 `--show-args` 查看 launch 文件支持的全部参数，例如：

```bash
ros2 launch dart_bringup camera_system.launch.py --show-args
```

### 电控无反馈时模拟运行

电控暂未发送数据时，可以直接发布模拟 ROS 话题，运行视觉和瞄准流程。
`/controller_state` 提供目标模式、飞镖偏角和电机 yaw，`/joint_states` 提供发射架关节角以更新 TF。
`/controller_logger` 仅用于调试和录包，不能替代这两个输入。

在工作空间根目录打开以下终端，每个终端均需加载环境。终端 1 启动视觉系统并关闭串口节点：

```bash
source install/setup.bash
ros2 launch dart_bringup vision_system.launch.py start_serial:=false
```

终端 2 以 50 Hz 发布控制器状态，模拟前哨站模式、零飞镖偏角和零电机 yaw：

```bash
source install/setup.bash
ros2 topic pub -r 50 /controller_state \
  dart_interfaces/msg/ControllerState \
  "{header: auto, target_mode: 1, dart_offset_rad: 0.0, launcher_yaw_rad: 0.0}"
```

`header: auto` 会在每次发布时填入当前时间，避免控制器状态因时间戳过期而失效。
本例使用默认的系统时间，不设置 `use_sim_time:=true`。

终端 3 以 50 Hz 发布发射架关节角：

```bash
source install/setup.bash
ros2 topic pub -r 50 /joint_states \
  sensor_msgs/msg/JointState \
  "{header: auto, name: ['launcher_yaw_joint'], position: [0.0]}"
```

角度单位均为弧度。当前串口配置的方向系数为 1、零点为 0，因此 `position` 与
`launcher_yaw_rad` 保持一致。若修改了这两个配置，模拟关节角应按
`motor_to_joint_sign * (launcher_yaw_rad - motor_zero_rad)` 计算。

终端 4 查看瞄准输出：

```bash
source install/setup.bash
ros2 topic echo /aim_command
```

有效瞄准输出仍需要相机检测、双目结果和 TF 正常。此方式绕过串口，不验证分帧或 CRC，
也不会向电控发送瞄准指令。若要验证串口链路，应使用虚拟串口发送协议帧。
恢复真实电控反馈前，先用 Ctrl+C 停止两个模拟发布器，再重新启动带串口节点的系统，
避免真实与模拟数据同时发布。

### 数据录制

项目使用 ROS 2 自带的 rosbag2 录制运行数据，配置统一保存在
`src/dart_bringup/config/rosbag/recording.yaml`。提供两种录制模式：

| 模式 | 内容 | 用途 |
| --- | --- | --- |
| `inputs` | 上下相机的压缩图像、CameraInfo、控制器反馈、动态和静态 TF | 后续代码测试与算法重算 |
| `full`（默认） | `inputs` 加检测、双目目标、瞄准指令、关节状态、诊断、日志和参数事件 | 完整历史复盘 |

录制默认关闭；启动完整视觉链路并启用完整录制：

```bash
ros2 launch dart_bringup vision_system.launch.py \
  start_recording:=true \
  recording_profile:=full
```

只录制后续代码测试所需的输入（需已有运行中的数据源）：

```bash
ros2 launch dart_bringup recording.launch.py profile:=inputs
```

每次录制默认保存在项目根目录的 `rosbag/dart_vision_YYYYMMDD_HHMMSS/`，其中 `bag/`
保存 MCAP，`config/` 保存配置基线副本及节点实际参数，`manifest.yaml` 记录 profile、话题、Git
版本和退出状态。快照保留配置目录结构，录制配置位于 `config/rosbag/recording.yaml`；
`snapshot_root` 相对于录制配置文件定位快照源根目录。上下相机的压缩图像用于持续复盘，但有损
压缩不适合作为严格的逐像素算法基准。
配置文件副本仅供参考；`config/runtime_parameters.yaml` 保存通过节点参数服务读取的实际值
（包含 launch 覆盖），`config/parameter_events.jsonl` 持续保存参数变更事件。manifest 的
`runtime_snapshot` 列出已抓取节点及错误；未开放参数服务的节点不能抓取。重算录制会在
所有必需节点的参数抓取完成后开始播放。标定消息等非参数状态仍以包内数据为准。
`rosbag/` 已被 Git 忽略，也可通过 `DART_BAG_ROOT` 临时覆盖保存根目录。录制按 30 秒或
4 GiB 中先达到的条件滚动分片；直接下电后，下一次启动录制器会隔离无法读取的分片并对
其余 MCAP 自动执行 `ros2 bag reindex`。最后一个未完成分片仍可能丢失。异常录制会保留
恢复标记，旧版遗漏标记的 failed 会话也会尝试恢复；恢复会将原 metadata
和不可读分片移至 session 的 `lost+found/`，可人工检查。录制阶段不会自动
删除或压缩历史数据，应在运行前确认磁盘剩余空间；执行 `git clean -fdx` 或删除工作区也会
删除被忽略的录包。

### 旧包裁剪与重算

含上下相机话题的录包可以只保留两路压缩图像、CameraInfo、控制器状态和 TF，使用当前程序重新
计算检测、双目目标和瞄准指令。先构建并加载当前工作区，然后生成新的 MCAP：

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run dart_bringup prepare_replay_bag.py \
  rosbag/dart_vision_20261015_000620 \
  rosbag/dart_vision_20261015_000620_replay
```

输出目录必须不存在，原包保留。保留列表统一使用 `src/dart_bringup/config/rosbag/recording.yaml`
的 `profiles.inputs`（支持 `include` 继承），录制、裁剪和重算回放共用同一份输入定义。
图像内容、消息时间戳和 TF 不变；旧算法结果、日志、诊断和冗余关节状态不写入新包。
`/controller_state` 使用自定义接口，需确保旧包和当前版本的消息结构兼容。

```bash
ros2 launch dart_bringup replay.launch.py \
  bag:=rosbag/dart_vision_20261015_000620_replay
```

默认 `mode:=recompute`，该入口启动上下相机的图像解码、当前检测/双目/瞄准节点，并用 `--clock-topics-all` 在每条消息前推进时钟，所有计算节点
使用仿真时间。相机、串口、机器人描述和自动录制均关闭，TF 来自录包。默认等待 3 秒
供节点发现，可通过 `delay:=5.0` 调整；机器处理不及时可用 `rate:=0.5` 降速。
瞄准节点仅在仿真时间下容忍最多 10 ms 的跨话题时钟传输偏差，实机仍严格拒绝未来时间戳。
双目队列与瞄准缓存通过实际 ROS 时钟回退事件清理，普通消息乱序不会清空双目队列。
默认循环播放，到包末尾后自动从头开始，按 Ctrl+C 停止整个 launch。
只播放一次可添加 `loop:=false`，播放结束后计算节点仍运行，按 Ctrl+C 退出。
也可直接指定原包，此入口仍只播放输入白名单。

查看录制时的全部信息，使用 `mode:=recorded`：

```bash
ros2 launch dart_bringup replay.launch.py \
  bag:=rosbag/dart_vision_20261015_000620 mode:=recorded
```

历史模式按包内实际话题播放，包含旧版独有的话题，不使用当前录制白名单过滤；按包中实际存在的上下相机压缩图像话题启动
对应的解码器，不启动当前检测、双目或瞄准算法。`inputs` 包也可历史回放，
但其中没有历史算法结果。两种模式都读取整个分片目录，不需要先合并或裁剪，均支持
`start`、`end`、`rate` 和 `loop`。历史自定义消息的查看仍需要兼容的接口定义。
回放入口会提示可检测到的 `.msg` 字段差异或缺失接口，其他类型问题由 rosbag2 报告；
区段导出保留原始消息描述和数据，不会自动将旧消息转换成新结构。历史模式也不要同时运行真实串口等执行节点。

用 `start` 和 `end` 选择相对于包起点的秒数，例如循环回放第 10～20 秒：

```bash
ros2 launch dart_bringup replay.launch.py \
  bag:=rosbag/dart_vision_20261015_000620_replay \
  start:=10.0 end:=20.0 rate:=0.5
```

`start` 默认 0，`end` 默认 -1（直到包末尾）；添加 `loop:=false` 只播放所选区段一次。
时间区段按录包时间计算，不受 `rate` 影响。指定区段会先生成临时 MCAP，保留原消息时间戳，
并补入区段起点前的静态 TF、最近的 CameraInfo 和控制器状态，以及动态 TF 最新状态和
起点前 0.5 秒内的历史。初始化消息的包接收时间设在区段起点前 1 ns，确保先于边界处的新状态，消息内部原始时间戳保留，
因此初始化数据可能来自区段外；后续消息按所选区段播放。退出后清理临时包，原始包不变。
循环用于重复观察，节点不会每轮重启，不保证各轮结果严格一致。

重算时同时保存输入和当前算法结果：

```bash
ros2 launch dart_bringup replay.launch.py \
  bag:=rosbag/dart_vision_20261015_000620 mode:=recompute \
  start_recording:=true recording_profile:=full
```

启用录制后 `loop` 默认变为 false，显式设置 `loop:=true` 会报错。录制只支持 `recompute`，
`recording_profile` 可选 `full`（含新结果）或 `inputs`。播放器先暂停，等待录制器订阅所有
回放输入及所选 profile 的算法输出，并确认必需节点和实际参数快照就绪后继续；录制器使用仿真时间。暂停期间先发布区段起点时钟，使录制器完成发现。播放结束后等待 1 秒收尾，自动关闭录制并退出 launch，
也可 Ctrl+C 提前结束。新 session 保存在现有录制根目录，可用 `DART_BAG_ROOT` 覆盖。
`manifest.yaml` 的 `replay` 字段保存源包路径、元数据摘要、源 manifest（如存在）、区段、
倍速和运行开关；`config/` 保存配置基线与实际参数。`storage_status` 表示文件关闭状态，
`replay_outcome` 表示重算结果：正常结束为 complete，Ctrl+C 为 interrupted，节点或播放器
失败为 failed。必需节点异常退出或播放器报错会令 launch 返回非零退出码；已正常关闭的
失败任务包仍可读取，不会当作损坏文件反复恢复。可通过 `recording_config_file:=...` 指定
录制与输入白名单共用的配置。录制以 ROS 实际收到的消息为准，
不是对源包逐字节复制，也不会恢复源包中未播放的历史算法结果。

离线回归检查（加载工作区后）：`python3 -m unittest discover -s src/dart_bringup/test -v`。

## ROS 包

| 包 | 职责 |
| --- | --- |
| `dart_interfaces` | 定义检测、双目目标、控制器状态和瞄准指令消息 |
| `dart_camera` | 分割绿色区域、筛选圆形目标，并计算和滤波单位视线 |
| `dart_stereo` | 配对上下相机检测结果，使用 TF 在配置参考系中进行三角测量 |
| `dart_aiming` | 将目标变换到发射架坐标系，执行函数拟合和偏角补偿 |
| `dart_serial` | 解析控制器数据、发布关节状态并发送瞄准指令 |
| `dart_bringup` | 集中管理项目 launch 文件和运行配置 |

`third_party/` 中包含以下子模块：

- `hik_camera_driver`：Hikrobot 工业相机驱动。
- `dart_description`：机器人模型和相机、发射架坐标系。
- `livox_ros2_driver`：Livox 雷达驱动。
- `stereo_calib`：以 `camera_1/camera_2` 编号的通用离线双目标定工具。

## 配置

第一方运行配置统一保存在 `src/dart_bringup/config/`：

| 路径 | 内容 |
| --- | --- |
| `camera/cameras.yaml` | 相机启用状态、序列号、曝光、帧率及标定文件引用 |
| `camera/stereo_capture.yaml` | 双相机标定采集的话题映射、配对时差、新鲜度及预览设置 |
| `camera/calibration/*.yaml` | 各相机内参和畸变参数 |
| `green_light_detector.yaml` | 按目标模式切换的颜色分割、几何、亮度和掩膜清理参数 |
| `stereo_triangulator.yaml` | 上下相机话题、配对时差和双目几何限制 |
| `aiming.yaml` | 超时、目标模式及二次函数拟合模型 |
| `serial.yaml` | 串口设备、波特率、话题和电机角度校正 |
| `lidar/` | Livox 驱动与设备配置 |

`aiming.yaml` 中的 `fitting.models` 可配置多个模型，并通过 `modes` 选择适用的目标模式。
`yaw_coefficients` 和 `distance_coefficients` 均按 `[a, b, c]` 配置，直接输出
`a*x^2 + b*x + c`；yaw 使用几何 yaw 作为输入，distance 使用几何距离作为输入。

`green_light_detector.yaml` 中的 `profiles` 将识别参数映射到控制器 `target_mode`：`outpost`
用于模式 `0`，`base` 用于模式 `1` 至 `4`。上下相机检测节点会订阅 `/controller_state` 并自动
切换 profile；未收到受支持的模式前不执行识别。

每套 profile 的 `center.method` 可独立选择绿灯观测中心算法，默认保留
`brightness_weighted`。候选分割、半径/亮度筛选、圆拟合得分和多候选消歧共用原有逻辑，
只改变最终 `center_x_px/center_y_px` 及由它们计算的单位视线。

| `center.method` | 计算方式 | 对照测试时关注 |
| --- | --- | --- |
| `brightness_weighted` | 轮廓内 G 通道减去局部背景，负权重置零后计算质心 | 默认方法；受内部亮度分布和饱和影响 |
| `contour_centroid` | 外轮廓围成区域的面积质心（多边形矩） | 不使用内部亮度权重；受分割边界影响 |
| `enclosing_circle` | 最小外接圆中心 | 速度快；容易被边界突起拉动 |
| `circle_fit` | 对完整像素边界做代数最小二乘圆拟合 | 利用整圈边界；不是抗离群点拟合 |
| `ellipse_fit` | 对完整像素边界调用 OpenCV `fitEllipse` | 允许椭圆形状；小目标或不完整边界可能不稳定 |

拟合中心时恢复逐像素边界，筛选用的简化轮廓不变。算法退化、拟合失败或中心越出轮廓
包围框时回退到已验证的最小外接圆中心；`/diagnostics` 的 `center_method` 显示当前激活
profile 的配置，`center_fallback_total` 累计统计有效检测中使用回退中心的次数。
切换方法不会重置此计数，应比较测试区间内的增量。

可在运行时切换，无需重启。基地模式（1～4）例如：

```bash
ros2 param set /top_camera/green_light_detector profiles.base.center.method circle_fit
ros2 param set /bottom_camera/green_light_detector profiles.base.center.method circle_fit
```

前哨站模式 0 使用 `profiles.outpost.center.method`。两个节点的设置不是跨节点原子操作，
应等两路均设置成功后再采样。运行时改参不会写回 YAML；持久配置修改
`src/dart_bringup/config/green_light_detector.yaml` 中对应的 `center.method`。
回放比较使用 `mode:=recompute`，`mode:=recorded` 不会重算旧检测结果。
不同方法可能有固定中心偏移，之前按亮度加权中心拟合的临时内外参不能直接作为其他方法的
精度依据；比较时同时记录均值偏移、1 秒窗口稳定性、检测率和回退计数。

检测节点默认对像素中心应用 EMA，再由滤波中心计算单位视线。`center_filter` 为每个
相机节点共用的配置，适用于所有目标 profile：

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `center_filter.enabled` | `true` | 是否启用 EMA；关闭时 `detection` 发布原始中心及对应视线 |
| `center_filter.tau_x_s` | `1.0` | X 方向时间常数，单位秒，必须有限且大于 0 |
| `center_filter.tau_y_s` | `1.0` | Y 方向时间常数，单位秒，必须有限且大于 0 |
| `center_filter.reset_gap_s` | `2.0` | 相邻有效图像间隔超过此值时重新初始化，必须有限且大于 0 |

滤波使用图像时间戳的实际间隔 `dt`，每轴计算 `alpha = 1 - exp(-dt / tau)` 和
`center = previous + alpha * (observation - previous)`，首个有效点直接作为初值。
默认时间常数适用于相机和目标基本静止的工况，理想阶跃约 3 秒达到变化量的 95%；
EMA 能减小短时抖动，但不能消除缓慢漂移。上下相机应采用相同参数，运动工况需要重新评估
响应滞后及双目误差。消息仍保留原图时间戳，处理耗时指标不包含滤波引入的响应滞后。

任一非 `DETECTED` 帧、目标模式变化（包括同一 profile 内的模式切换）、相机内参或图像
几何变化、重复/倒退时间戳和超时都会重置历史。修改当前 profile 的检测参数或改变 EMA
配置也会重置历史；重捕获首帧直接输出新中心，不延用旧目标，也不在丢失期间输出旧坐标。
仅靠有效检测无法判断未伴随模式变化的目标身份切换。

这些 EMA 参数支持运行时修改。例如暂时关闭上相机滤波用于对照：

```bash
ros2 param set /top_camera/green_light_detector center_filter.enabled false
```

持久设置写入 `src/dart_bringup/config/green_light_detector.yaml`。对照中心算法或采集原始
抖动时，将 `center_filter.enabled` 设为 `false`。检测结果统一通过 `detection` 发布。

当前相机清单将 `00DA1923275` 配置为上相机，将 `00DA1923281` 配置为下相机。
部署前必须确认实机序列号、上下安装位置和对应标定文件一致。

相机内参的图像尺寸必须与实际发布分辨率一致。双目外参来自 `dart_description` 发布的 TF；
修改相机安装位置后，应同步更新机器人描述中的安装变换，而不是在双目 YAML 中添加独立外参。
上下安装要求 TF 提供 `launcher_frame` 到 `top_camera_optical_frame`、
`bottom_camera_optical_frame` 的真实光心位置与朝向。当前 `dart_description` 使用
`vertical-stereo` 分支，两台相机直接安装在 `launcher_frame` 下；模型光心间距约 0.47096 m，
同时存在前后偏移和高度差，安装外参仍需按实机标定复核。
相机或图像旋转由实际光学坐标关系决定，不交换检测中心的 x/y。
仅移动相机且镜头、焦距和原始图像模式未变时，可复用并复核单目内参；双目外参需要重新确定。
标定工具使用 `camera_1/camera_2`，当前系统分别映射到上相机 `00DA1923275` 和下相机
`00DA1923281`。使用安装约束时，需在 `third_party/stereo_calib/configs/installation.json`
填写两个光学坐标系相对公共参考系（例如 `launcher_frame`）的理论位姿；位置单位为 mm，
不能直接填安装关节的位置。该工具的默认配置是通用示例，部署前必须按实际安装替换。
机器人描述默认发布机械模型；场景分析使用 `dart_description` 的 `mode:=scene team:=red/blue`
入口。安装偏移定义在 URDF 中，主项目已移除不再生效的 `site_file` 入口及场地 YAML。

## 双相机标定采集

使用配对采集窗口替代两路 rqt 手动保存。该入口只启动相机驱动和采集工具：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dart_bringup
source install/setup.bash
ros2 launch dart_bringup stereo_capture.launch.py
```

窗口中 `S` 请求保存一组新图像，`T` 选择训练集，`V` 选择验证集，`Q` 或 Esc 退出。
先摆好标定板并保持相机与板稳定，再按 `S`；请求只接受按键之后的新帧，缓存中的旧帧不会被保存。
当前默认要求两路图像时间差不超过 10 ms、年龄不超过 200 ms，等待 3 秒仍未凑齐则整组失败。
窗口显示两路图像年龄、主机收帧时间差、当前划分、保存数量及失败原因。

相机驱动已在其他入口运行时，使用 `start_driver:=false` 避免重复连接。可通过
`output_root:=/absolute/path` 指定保存根目录，`params_file:=...` 覆盖采集参数；相机驱动清单
通过 `cameras_file:=...` 指定。默认配置将 `camera_1` 映射到上相机、`camera_2` 映射到下相机。
相机保持 `trigger_mode: off`，曝光和增益配置来自驱动清单。

每次运行创建独立目录，默认位于运行目录下的 `captures/stereo_YYYYMMDD_HHMMSS_<id>/`：

```text
camera_1/000001.png
camera_2/000001.png
metadata/000001.json
pairs.csv
session.json
```

PNG 保留原始分辨率和 8-bit 像素，窗口缩放不影响保存结果；RGB 输入仅转换通道顺序后写入 PNG。
两张图片和元数据写入成功后才更新 `pairs.csv`，失败不会生成半组清单记录，也不会覆盖已有图片。
同一 session 的分辨率、编码或 frame ID 改变时拒绝保存，需启动新 session。

元数据保存原图时间戳、收帧时间差、相机映射、可用的 CameraInfo，以及采样时的驱动曝光、增益等
参数。驱动参数是异步快照，保存其采样时间；它不是逐帧曝光事件记录。默认要求能够读取驱动参数，
并核对配置的序列号；外部图像源没有参数服务时，可在自定义 YAML 中设置
`require_driver_parameters: false`。采集不要求已有有效 K/D，尚未标定的 CameraInfo 可以正常保存。

无图形界面时关闭预览，通过服务请求保存；服务返回的是实际保存结果：

```bash
ros2 launch dart_bringup stereo_capture.launch.py preview_enabled:=false
ros2 service call /stereo_capture/capture std_srvs/srv/Trigger "{}"
ros2 param set /stereo_capture split validation
```

当前实现使用自由运行图像的主机收帧时间配对，不能据此保证曝光同步；适用于静止标定采集。
运动标定仍需要协调软件触发或共同硬件触发。训练和验证应分别采集独立板姿态，避免把相邻帧随机
分到两组。

退出采集后，在 `stereo_calib` 目录中使用该 session 的清单，生成独立的观测和模型：

```bash
cd third_party/stereo_calib
CAPTURE_SESSION=/absolute/path/to/captures/stereo_YYYYMMDD_HHMMSS_id
python3 -m stereo_calib detect --manifest "$CAPTURE_SESSION/pairs.csv" \
  --output "$CAPTURE_SESSION/observations.json" --previews "$CAPTURE_SESSION/detections"
python3 -m stereo_calib calibrate-mono --camera camera_1 \
  --observations "$CAPTURE_SESSION/observations.json" --output "$CAPTURE_SESSION/camera_1.json"
python3 -m stereo_calib calibrate-mono --camera camera_2 \
  --observations "$CAPTURE_SESSION/observations.json" --output "$CAPTURE_SESSION/camera_2.json"
python3 -m stereo_calib calibrate-stereo --observations "$CAPTURE_SESSION/observations.json" \
  --camera-1-model "$CAPTURE_SESSION/camera_1.json" --camera-2-model "$CAPTURE_SESSION/camera_2.json" \
  --output "$CAPTURE_SESSION/stereo.json"
python3 -m stereo_calib validate --observations "$CAPTURE_SESSION/observations.json" \
  --stereo "$CAPTURE_SESSION/stereo.json" --output "$CAPTURE_SESSION/validation.json"
```

## 主要话题与接口

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/top_camera/image_raw`、`/bottom_camera/image_raw` | `sensor_msgs/msg/Image` | 上下相机的原始图像 |
| `/top_camera/camera_info`、`/bottom_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 与图像同时间戳的相机参数 |
| `/top_camera/detection`、`/bottom_camera/detection` | `dart_interfaces/msg/GreenLightDetection` | EMA 后的绿灯中心及对应单位视线，供双目节点使用；关闭 EMA 时为原始值 |
| `/stereo_target` | `dart_interfaces/msg/StereoTarget` | 默认在 `launcher_frame` 中的三维目标，实际参考系见消息头 |
| `/controller_logger` | `dart_interfaces/msg/ControllerLogger` | 下位机日志包完整业务字段及主机接收时间戳 |
| `/controller_state` | `dart_interfaces/msg/ControllerState` | 目标模式、飞镖偏角和电机反馈角 |
| `/aim_command` | `dart_interfaces/msg/AimCommand` | 默认相对 `launcher_frame` 的偏转角、水平距离和有符号垂直高度差 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 校正后的发射架关节角 |

消息中的 `center_x_px`、`center_y_px` 为输入图像坐标系中未去畸变的亚像素中心，
仅在 `status == DETECTED` 时有效，其余状态为 0；不要用坐标是否为零判断有效性。
`unit_ray` 始终由消息中的中心计算。录包回放需要使用与包内数据一致的
消息类型定义；旧包中的 `detection` 可能为原始值，应结合采集时的配置解释。

坐标约定：相机光学坐标系为 x 向右、y 向下、z 向前；双目中心和发射架参考系为
x 向前、y 向左、z 向上。所有 yaw 和串口角度均使用弧度，并以向左为正。

双目使用三维视线最近点中点估计绿灯中心。
该算法支持水平、竖直及斜向基线；两路输入参数统一使用 `top_*`、`bottom_*`。
`StereoTarget.ray_gap_m` 为两条空间视线最近点的间距；`min_ray_angle_deg` 限制三维交会锐角，
同时排除平行和反平行退化；`max_camera_range_m` 限制估计点到任一相机光心的空间距离。
`max_ray_gap_m` 限制两条视线最近点的间距，默认 0.1 m，需根据实机数据验证。
双目 `position` 是三维估计，`distance` 和 `yaw` 是其在双目参考系中的派生量。

双目和瞄准当前默认使用 `launcher_frame`：距离和高度差均相对发射架原点。
如果下位机需要相对离轨点输出，将 `aiming.yaml` 的 `reference_frame` 改为
`ballistic_reference_frame`，节点会按测量时间转换目标位置；该坐标系原点与
`dart_release_link` 一致，轴与 `launcher_frame` 对齐，不带离轨方向的俯仰角。
当前描述中该原点相对发射架偏移为 `[0.2, 0, 0.6]` m，切换参考系会改变输出距离和高度差。
当前模型假设底座水平且机构只有 yaw 转动；
底座倾斜时须提供重力对齐的参考系。`horizontal_distance_m = hypot(x, y)`（可经过距离校正，
当前系数为恒等映射），`vertical_difference_m = z`，向上为正，允许负数和零。
测量目标为绿灯中心；高度差原样透传，yaw 补偿只作用于偏转角。
三维估计不依赖前哨站或基地的预设高度，发射能量映射由下位机使用两个距离分量处理。

### 串口发送协议

发送帧头为 `0xA5`，帧长度为 16 字节，下位机按下表解析字段和校验 CRC。
普通控制器反馈帧头为 `0x5A`，控制器日志包帧头为 `0xD5`。

| 字节偏移 | 字段 | 编码与含义 |
| --- | --- | --- |
| 0 | header | `0xA5` |
| 1 | state | `0=CLOSED`、`1=VALID`、`2=INVALID` |
| 2–5 | yaw_rad | IEEE 754 float32，小端，弧度，向左为正 |
| 6–9 | horizontal_distance_m | float32 小端，瞄准参考系原点到绿灯的水平距离，米 |
| 10–13 | vertical_difference_m | float32 小端，绿灯相对瞄准参考系原点的垂直高度差，米，向上为正 |
| 14–15 | CRC16 | 前 14 字节的校验，小端；初值 `0xffff`，反射多项式 `0x8408`，无最终异或 |

只有 `VALID` 的几何字段有效，其他状态下三个浮点字段全部清零。
输入重算要求包内 TF 包含所配置的双目与瞄准参考系以及上下相机光学坐标系，
变换必须与录制时的安装一致。当前回放入口使用上下相机话题；旧左右相机录包需使用匹配的旧版
配置和回放入口，不能仅通过重命名话题将旧安装数据当作新安装数据。
PlotJuggler 布局提供偏转角、水平距离和垂直高度差曲线。

视觉派生消息始终保留原始测量时间：单目检测继承图像时间戳，双目目标使用上下图像
时间戳的中间时刻，瞄准指令继续继承该双目测量时间。定时重复发布不会刷新时间戳；没有
任何观测来源的 `INVALID` 指令使用零时间戳。因此串口的新鲜度检查衡量的是完整视觉链路
的数据年龄，而不是瞄准节点最近一次发布时间。

可以通过以下命令观察关键输出：

```bash
ros2 topic echo /top_camera/detection
ros2 topic echo /stereo_target
ros2 topic echo /aim_command
```

## 日志与诊断

运行时事件通过 `/rosout` 输出，各运行节点通过 `/diagnostics` 以 1 Hz 发布状态：

```bash
ros2 topic echo /rosout
ros2 topic echo /diagnostics
```

诊断内容覆盖相机连接与丢包、单目处理频率、双目配对和几何拒绝、瞄准失效原因、串口连接
与协议错误。视觉各阶段使用消息携带的 T0 统计累计延迟；最近 200 个有效样本分别给出
平均值、最大值和 P95：

- `capture_to_detection_*_ms`：T0 到单目检测和 EMA 输出，不包含滤波响应滞后；
- `measurement_to_stereo_*_ms`：T0 到双目输出；
- `measurement_to_aim_*_ms`：T0 到瞄准结果；
- `measurement_to_serial_write_*_ms`：T0 到有效指令写入串口完成。

双目诊断还发布上下相机输入频率、配对频率、配对时间差、未配对计数和各几何拒绝原因计数；
瞄准诊断发布当前输出状态及最近失效原因；串口诊断发布收发频率、最近收发时间、CRC、
解码、读写和重连计数。延迟暂不设置固定告警阈值，应先根据实机数据确定正常范围。

节点启动、模式切换和串口连接使用 `INFO`；持续影响输出的 TF 和连接问题使用
`WARN`，最多每 5 秒提醒一次；未配对观测丢弃和逐帧三角测量拒绝详情使用 `DEBUG`，最多每秒一次。
未配对观测仍计入诊断计数，缺少输入或没有配对时由 `/diagnostics` 报告告警；串口
协议错误最多每 2 秒一次，串口重连失败最多每 10 秒一次。高频测量数据应通过 rosbag
记录，不通过日志逐帧输出。

## 状态语义

| 阶段 | 有效状态 | 其他状态 |
| --- | --- | --- |
| 单目检测 | `CLOSED=0`、`DETECTED=1` | `NO_TARGET=2`、`ERROR=3` |
| 双目目标 | `CLOSED=0`、`VALID=1` | `INVALID=2` |
| 瞄准指令 | `CLOSED=0`、`VALID=1` | `INVALID=2` |

- `CLOSED` 表示清理后的绿色掩膜中没有轮廓，由视觉结果推断舱门关闭。
- 双目仅在上下相机都为 `DETECTED` 且 TF 与几何检查通过时产生 `VALID`。
- 瞄准仅在目标和控制器状态未超时且 TF 可用时产生 `VALID`。
- 非有效状态下的位置、角度和距离字段均为零。
- 串口线协议使用 `0=CLOSED`、`1=VALID`、`2=INVALID`；下位机必须使用相同语义。

完全遮挡、绿灯熄灭或绿色分割失败且没有残留轮廓时，也可能被判断为 `CLOSED`。仅靠当前
视觉判据无法区分这些情况与真实关门；需要严格区分时应引入独立舱门传感器或门体识别。

## 代码检查

Python 环境仅用于 launch 文件的格式与静态检查，不参与运行时算法：

```bash
uv sync --python /usr/bin/python3
uv run ruff format --check src
uv run ruff check src
```

Jupyter 数据分析工具位于 `analysis` 依赖组，不会随默认开发环境安装：

```bash
uv sync --python /usr/bin/python3 --group analysis
uv run --group analysis jupyter lab analysis/notebooks/target_impact_analysis.ipynb
```

完整执行 notebook 并重新生成报告与四张图片（需先加载 ROS 的 Xacro 环境）：

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/python analysis/run_target_impact_analysis.py --export-report
.venv/bin/python -m unittest discover -s analysis -p test_stereo_precision.py -v
```

该 notebook 从 `dart_description` 的 URDF 读取离轨点、固定发射方向和装甲板上表面，分析
绿灯到装甲板的常量 yaw 补偿可行性、滑轨全行程瞄准结果、固定目标时 yaw 误差造成的
板面撞击位移，以及视觉角度和双目距离精度。板面使用 `u/v` 坐标，其中 `u` 对应
`armor_link` local `y`，`v` 对应 local `z`；所有几何参数以当前 URDF 为准。
像素噪声分析读取两台相机各自的 K/D，通过完整三维安装变换和空间射线求交传播到
`launcher_frame` 中的 yaw、水平距离和高度差，不采用平行双目的水平视差近似。
计算结果、图表、适用边界和工程建议汇总在
[`飞镖视觉瞄准几何与误差传播分析报告`](docs/reports/dart_vision_aiming_geometry_error_analysis/report.md)。

构建并加载工作区后，可运行包含真实描述模型、瞄准参考系转换和 PTY 串口的端到端回归：

```bash
source install/setup.bash
python3 -m unittest discover -s src/dart_bringup/test -p test_geometry_pipeline.py -v
```

Python 直接依赖统一声明在 `pyproject.toml`，完整解析版本由 `uv.lock` 锁定。修改依赖时使用
`uv add --group dev <package>` 或 `uv add --group analysis <package>`，并提交同步更新的两个文件。

C++ 格式和静态分析规则分别位于 `.clang-format` 和 `.clang-tidy`。

## License

本项目使用 [MIT License](LICENSE)。第三方子模块遵循各自仓库中的许可证。

### 实时查看下位机日志

运行 `vision_system.launch.py` 时，串口节点将每个校验通过的 `0xD5` 日志包发布到
`/controller_logger`，无需开启 DEBUG。协议对齐电控端的 24 字节 packed 格式，包含 13 个单字节状态字段、2 个 float32 拉力值（kg）、
帧头和 CRC16；没有 vision_stable_state 字段。消息包含全部业务字段；帧头和 CRC 不作为业务字段发布。
时间戳为主机解析有效帧时的 ROS 时间，并非下位机采样时间。原有 DEBUG 文本仍按 2 秒限频输出，
话题发布不受该限频影响。

编译更新并重新启动系统：

```bash
colcon build --packages-select dart_interfaces dart_serial dart_bringup
source install/setup.bash
ros2 launch dart_bringup vision_system.launch.py
```

另开终端，在工作空间中执行：

```bash
source install/setup.bash
ros2 topic echo /controller_logger --qos-reliability best_effort
# 只查看左侧拉力：
ros2 topic echo /controller_logger --field string_l_force --qos-reliability best_effort
```

话题使用 SensorDataQoS（best effort、深度 5），优先提供最新数据；慢订阅端可能丢帧。
实际频率取决于下位机日志发送频率。若没有输出，可查看 `/diagnostics` 中的
`logger_frames_total` 是否增长。默认 `full` 录包配置已包含该话题，
通过 `start_recording:=true` 启用；`inputs` 配置不包含日志话题。
不要额外启动第二个串口节点。
