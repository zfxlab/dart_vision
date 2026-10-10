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
  python3-colcon-common-extensions python3-rosdep pipx
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
| `/left_camera/detection`、`/right_camera/detection` | `dart_interfaces/msg/GreenLightDetection` | 相机光学坐标系中的原始目标单位视线 |
| `/left_camera/detection_filtered`、`/right_camera/detection_filtered` | `dart_interfaces/msg/GreenLightDetection` | Kalman 滤波后的目标单位视线，供双目节点使用 |
| `/stereo_target` | `dart_interfaces/msg/StereoTarget` | 双目中心坐标系中的三维目标 |
| `/controller_state` | `dart_interfaces/msg/ControllerState` | 目标模式、飞镖偏角和电机反馈角 |
| `/aim_command` | `dart_interfaces/msg/AimCommand` | 发射架坐标系中的偏转角和水平距离 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 校正后的发射架关节角 |

坐标约定：相机光学坐标系为 x 向右、y 向下、z 向前；双目中心和发射架参考系为
x 向前、y 向左、z 向上。所有 yaw 和串口角度均使用弧度，并以向左为正。

可以通过以下命令观察关键输出：

```bash
ros2 topic echo /left_camera/detection
ros2 topic echo /stereo_target
ros2 topic echo /aim_command
```

## 日志与诊断

运行时事件通过 `/rosout` 输出，相机处理状态通过 `/diagnostics` 以 1 Hz 发布：

```bash
ros2 topic echo /rosout
ros2 topic echo /diagnostics
```

节点启动、模式切换和串口连接使用 `INFO`；持续影响输出的 TF、配对和连接问题使用
`WARN`，最多每 5 秒提醒一次；逐帧三角测量拒绝详情使用 `DEBUG`，最多每秒一次；串口
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
uv venv --python /usr/bin/python3
uv pip sync requirements-uv.lock
source .venv/bin/activate
ruff format --check src
ruff check src
```

C++ 格式和静态分析规则分别位于 `.clang-format` 和 `.clang-tidy`。

## License

本项目使用 [MIT License](LICENSE)。第三方子模块遵循各自仓库中的许可证。
