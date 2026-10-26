# Dart Vision

基于 ROS 2 Jazzy 的飞镖视觉与瞄准系统。项目从双海康工业相机获取图像，检测绿色目标，
通过双目几何计算目标位置，再结合控制器状态和 TF 生成瞄准指令；同时提供串口通信和
Livox 雷达调试入口。

```text
hik_camera_driver
        │ Image + CameraInfo
        ▼
dart_camera（左右绿灯检测）
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

### 数据录制

项目使用 ROS 2 自带的 rosbag2 录制运行数据，配置统一保存在
`src/dart_bringup/config/rosbag/recording.yaml`。提供两种录制模式：

| 模式 | 内容 | 用途 |
| --- | --- | --- |
| `inputs` | 左右压缩图像、CameraInfo、控制器反馈、动态和静态 TF | 后续代码测试与算法重算 |
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
`snapshot_root` 相对于录制配置文件定位快照源根目录。左右压缩图像用于持续复盘，但有损
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

旧版本录包可以只保留左右压缩图像、CameraInfo、控制器状态和 TF，使用当前程序重新
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

默认 `mode:=recompute`，该入口启动左右图像解码、当前检测/双目/瞄准节点，并用 `--clock-topics-all` 在每条消息前推进时钟，所有计算节点
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

历史模式按包内实际话题播放，包含旧版独有的话题，不使用当前录制白名单过滤；仅启动
已有左右压缩图像的解码器，不启动当前检测、双目或瞄准算法。`inputs` 包也可历史回放，
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
| `dart_stereo` | 配对左右检测结果，使用 TF 在双目中心坐标系中进行三角测量 |
| `dart_aiming` | 将目标变换到发射架坐标系，执行函数拟合和偏角补偿 |
| `dart_serial` | 解析控制器数据、发布关节状态并发送瞄准指令 |
| `dart_bringup` | 集中管理项目 launch 文件和运行配置 |

`third_party/` 中包含以下子模块：

- `hik_camera_driver`：Hikrobot 工业相机驱动。
- `dart_description`：机器人模型和相机、发射架坐标系。
- `livox_ros2_driver`：Livox 雷达驱动。

## 配置

第一方运行配置统一保存在 `src/dart_bringup/config/`：

| 路径 | 内容 |
| --- | --- |
| `camera/cameras.yaml` | 相机启用状态、序列号、曝光、帧率及标定文件引用 |
| `camera/calibration/*.yaml` | 各相机内参和畸变参数 |
| `green_light_detector.yaml` | 按目标模式切换的颜色分割、几何、亮度和掩膜清理参数 |
| `stereo_triangulator.yaml` | 左右话题、配对时差和双目几何限制 |
| `aiming.yaml` | 超时、目标模式及二次函数拟合模型 |
| `serial.yaml` | 串口设备、波特率、话题和电机角度校正 |
| `site/default.yaml` | 红蓝方选择及场地坐标修正 |
| `lidar/` | Livox 驱动与设备配置 |

`aiming.yaml` 中的 `fitting.models` 可配置多个模型，并通过 `modes` 选择适用的目标模式。
`yaw_coefficients` 和 `distance_coefficients` 均按 `[a, b, c]` 配置，直接输出
`a*x^2 + b*x + c`；yaw 使用几何 yaw 作为输入，distance 使用几何距离作为输入。

`green_light_detector.yaml` 中的 `profiles` 将识别参数映射到控制器 `target_mode`：`outpost`
用于模式 `0`，`base` 用于模式 `1` 至 `4`。左右检测节点会订阅 `/controller_state` 并自动
切换 profile；未收到受支持的模式前不执行识别。

当前相机清单将 `00DA1923275` 配置为左相机，将 `00DA1923281` 配置为右相机。
部署前必须确认实机序列号、左右安装位置和对应标定文件一致。

相机内参的图像尺寸必须与实际发布分辨率一致。双目外参来自 `dart_description` 发布的 TF；
修改相机安装位置后，应同步更新机器人描述中的安装变换，而不是在双目 YAML 中添加独立外参。

## 主要话题与接口

| Topic | 类型 | 说明 |
| --- | --- | --- |
| `/left_camera/image_raw`、`/right_camera/image_raw` | `sensor_msgs/msg/Image` | 左右原始图像 |
| `/left_camera/camera_info`、`/right_camera/camera_info` | `sensor_msgs/msg/CameraInfo` | 与图像同时间戳的相机参数 |
| `/left_camera/detection`、`/right_camera/detection` | `dart_interfaces/msg/GreenLightDetection` | 原始目标单位视线及绿灯中心像素坐标，供双目节点使用 |
| `/stereo_target` | `dart_interfaces/msg/StereoTarget` | 双目中心坐标系中的三维目标 |
| `/controller_state` | `dart_interfaces/msg/ControllerState` | 目标模式、飞镖偏角和电机反馈角 |
| `/aim_command` | `dart_interfaces/msg/AimCommand` | 离轨点水平参考系中的偏转角、水平距离和有符号垂直高度差 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 校正后的发射架关节角 |

检测节点不进行时间滤波。消息中的 `center_x_px`、`center_y_px` 为输入图像上未去畸变的亚像素中心，
仅在 `status == DETECTED` 时有效，其余状态为 0；不要用坐标是否为零判断有效性。
录包回放需要使用与包内数据一致的消息类型定义。
历史 Kalman 实验保留在 `analysis/labplot/`，离线回放使用独立的 `kalman_replay.yaml`。

坐标约定：相机光学坐标系为 x 向右、y 向下、z 向前；双目中心和发射架参考系为
x 向前、y 向左、z 向上。所有 yaw 和串口角度均使用弧度，并以向左为正。

双目使用三维视线最近点中点估计绿灯中心。
`StereoTarget.ray_gap_m` 为两条空间视线最近点的间距；`min_ray_angle_deg` 限制三维交会锐角，
同时排除平行和反平行退化；`max_camera_range_m` 限制估计点到任一相机光心的空间距离。
`max_ray_gap_m` 限制两条视线最近点的间距，默认 0.1 m，需根据实机数据验证。
双目 `position` 是三维估计，`distance` 和 `yaw` 是其在双目参考系中的派生量。

瞄准默认转换到 `ballistic_reference_frame`：原点与 `dart_release_link` 一致，轴与
`launcher_frame` 对齐，不带离轨方向的俯仰角。当前模型假设底座水平且机构只有 yaw 转动；
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
| 6–9 | horizontal_distance_m | float32 小端，离轨点到绿灯的水平距离，米 |
| 10–13 | vertical_difference_m | float32 小端，绿灯相对离轨点的垂直高度差，米，向上为正 |
| 14–15 | CRC16 | 前 14 字节的校验，小端；初值 `0xffff`，反射多项式 `0x8408`，无最终异或 |

只有 `VALID` 的几何字段有效，其他状态下三个浮点字段全部清零。
输入重算要求 TF 中包含与录制时离轨位置一致的 `ballistic_reference_frame`，
其轴方向与水平发射架参考系对齐。PlotJuggler 布局提供偏转角、水平距离和垂直高度差曲线。

视觉派生消息始终保留原始测量时间：单目检测继承图像时间戳，双目目标使用左右图像
时间戳的中间时刻，瞄准指令继续继承该双目测量时间。定时重复发布不会刷新时间戳；没有
任何观测来源的 `INVALID` 指令使用零时间戳。因此串口的新鲜度检查衡量的是完整视觉链路
的数据年龄，而不是瞄准节点最近一次发布时间。

可以通过以下命令观察关键输出：

```bash
ros2 topic echo /left_camera/detection
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

- `capture_to_detection_*_ms`：T0 到单目检测和 Kalman 输出；
- `measurement_to_stereo_*_ms`：T0 到双目输出；
- `measurement_to_aim_*_ms`：T0 到瞄准结果；
- `measurement_to_serial_write_*_ms`：T0 到有效指令写入串口完成。

双目诊断还发布左右输入频率、配对频率、配对时间差、未配对计数和各几何拒绝原因计数；
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
- 双目仅在左右相机都为 `DETECTED` 且 TF 与几何检查通过时产生 `VALID`。
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

该 notebook 从 `dart_description` 的 URDF 读取离轨点、固定发射方向和装甲板上表面，分析
绿灯到装甲板的常量 yaw 补偿可行性、滑轨全行程瞄准结果、固定目标时 yaw 误差造成的
板面撞击位移，以及视觉角度和双目距离精度。板面使用 `u/v` 坐标，其中 `u` 对应
`armor_link` local `y`，`v` 对应 local `z`；所有几何参数以当前 URDF 为准。
计算结果、图表、适用边界和工程建议汇总在
[`飞镖视觉瞄准几何与误差传播分析报告`](docs/reports/dart_vision_aiming_geometry_error_analysis/report.md)。

Python 直接依赖统一声明在 `pyproject.toml`，完整解析版本由 `uv.lock` 锁定。修改依赖时使用
`uv add --group dev <package>` 或 `uv add --group analysis <package>`，并提交同步更新的两个文件。

C++ 格式和静态分析规则分别位于 `.clang-format` 和 `.clang-tidy`。

## License

本项目使用 [MIT License](LICENSE)。第三方子模块遵循各自仓库中的许可证。
