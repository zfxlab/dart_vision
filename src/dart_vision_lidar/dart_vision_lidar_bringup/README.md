# Mid-70 点云定位部署说明

`dart_vision_lidar_bringup` 集中保存 Mid-70 基地 / 装甲板定位第一阶段的现场参数、launch、RViz 配置和部署手册。实时算法与模型离线构建彼此独立，主系统启动不会自动重建模型。

## 三包架构

| 包 | 职责 |
| --- | --- |
| `dart_vision_lidar_model` | PLY/STL 三角网格面积加权均匀采样、单位缩放、VoxelGrid、PCD 保存及普通 YAML 批量 builder |
| `dart_vision_lidar_localization` | C++ 实时输入适配、累积、预处理、基地状态/位姿、装甲板一维搜索、绿灯坐标和调试输出 |
| `dart_vision_lidar_bringup` | 本包：部署 YAML、启动入口、RViz 和标定说明 |

输出消息仍由工作区公共包 `dart_vision_interfaces` 提供。这样拆分后，模型工具不需要随实时节点启动，部署参数也不与算法库或原始 CAD 混在一起。

```text
PointCloud2 或 livox_interfaces/CustomMsg
  -> 转换并按输入时间戳变换到 lidar_frame
  -> 时间窗口多帧累积（帧数和点数双上限）
  -> NaN、距离、CropBox、VoxelGrid、可选 SOR
  -> 开/关固定基地独立模板的局部 ICP + 单向质量指标 + 时序门控
  -> 移动装甲板模板沿已知轴的一维粗/细搜索 + 时序门控
  -> armor 中固定绿灯偏移 -> lidar/output XYZ 和雷达原点距离
  -> Mid70Localization + 八路点云 + PoseStamped + MarkerArray
```

基地打开固定部分、基地关闭固定部分、移动装甲板必须使用三个独立模板。基地采用受限 `x/y/yaw` ICP，Z 平移和 roll/pitch 始终保持 YAML 初值；不会只凭 `hasConverged()` 判定成功，还会检查初值附近的位姿增量、截断最近邻残差、内点率、模板覆盖率、综合分数、开关分差和连续帧。装甲板不会在线自由六自由度漂移，也不会把完整 CAD 与不完整场景做强制双向全量匹配。

## 构建

当前工作区目标环境是 ROS 2 Humble：

```bash
cd /path/to/dart_vision
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to dart_vision_lidar_bringup
source install/setup.bash
```

构建不依赖现场模型路径；随包安装的示例 PCD 可用于启动和接口检查，最终参数仍须使用实物与 rosbag 验证。

## 离线生成三个模型

安装后的 `config/model_build_example.yaml` 是普通 YAML，不是 ROS 参数文件。复制到场地目录，替换三个明确拆分的 PLY 或 STL 路径，并确认每个网格已经位于对应的目标模板坐标系：

```bash
bringup_share="$(ros2 pkg prefix --share dart_vision_lidar_bringup)"
mkdir -p /data/mid70
cp "${bringup_share}/config/model_build_example.yaml" /data/mid70/models.yaml
# 编辑 /data/mid70/models.yaml；相对路径相对该 YAML 所在目录解析。
ros2 run dart_vision_lidar_model dart_vision_model_builder \
  --config /data/mid70/models.yaml
```

也可显式运行一次独立 builder launch：

```bash
ros2 launch dart_vision_lidar_bringup build_models.launch.py \
  config_file:=/data/mid70/models.yaml
```

该 launch 与 `mid70_localization.launch.py` 没有 include 关系；实时启动不会每次覆盖或重建 PCD。示例 YAML 每项包含 `name`、`input_mesh`、`output_pcd`、`scale_to_m`、采样点数、随机种子、体素大小和覆盖策略。PCD 固定以 binary 格式保存；`overwrite: false` 可避免误覆盖已验收模型。

三个已处理 PCD 随 `dart_vision_lidar_model` 安装。定位 launch 会从该包的 package share 自动解析模型路径并覆盖 YAML 中的空值，不依赖工作区或用户名。任一路径不可读或没有有限点时节点正常启动但持续发布 `valid=false`、`STATUS_MODEL_UNAVAILABLE`。

## 米制、frame 与变换方向

所有内部长度、PCD、ROI、搜索行程、阈值和距离统一为米；角度参数为弧度。离线 PLY/STL 输入必须已经位于目标模板 frame，builder 仅执行 `p_pcd_m = scale_to_m * p_mesh`，不执行刚体变换或中心化。在线定位统一记号：

```text
t_target_source：把 source 坐标的点变换到 target
p_target = t_target_source * p_source
t_output_base = t_output_lidar * t_lidar_base
t_lidar_armor = t_lidar_base * t_base_armor
p_lidar_green = t_lidar_armor * p_armor_green
```

- 输入 `header.frame_id` 是 `input`，节点查询该时间戳的 `t_lidar_input`，变换到 `lidar_frame` 后才累积。
- `base.initial_*` 定义 `t_lidar_base_initial`；RPY 顺序为 `Rz(yaw) * Ry(pitch) * Rx(roll)`。
- `armor.zero_*` 定义 `t_base_armor_zero`；`armor.motion_axis_base` 在 `base` 中表达并会归一化。
- `green.offset_armor_m` 是绿灯中心在 `armor` 坐标中的固定偏移。
- 结果总 header 和 `green_light_output` 使用 `output_frame`；`green_light_lidar`、八路调试云和 Marker 使用 `lidar_frame`。`green_light_distance_m` 是绿灯到 `lidar_frame` 原点的欧氏距离。

如果 `t_output_lidar` 暂时不可用，总结果必为无效并返回 `STATUS_OUTPUT_TF_UNAVAILABLE`。为保留诊断信息，有限的 `base_pose` / `armor_pose` 会回退为 `lidar_frame`，其各自 `PoseStamped.header.frame_id` 才是权威 frame；此时 `green_light_lidar` 和雷达距离仍可用于人工诊断，但 `green_light_output` 保持无效。下游控制不得把这个回退候选当作有效输出。

## 配置与可靠性语义

`config/mid70_localization.yaml` 与实时节点声明的参数逐项对应。建议复制为场地 YAML 后标定：`lidar_frame/output_frame`、基地初值、装甲板零位/轴/行程、绿灯偏移、雷达 ROI 和质量阈值。默认模型路径由 launch 解析；如需使用场地外部模型，通过三个模型 launch 参数显式覆盖。

累积同时受 `time_window_s`、`max_frames` 和 `max_points` 约束。时间戳倒退默认清空累积及连续帧状态；也可配置为拒绝倒退帧。SOR 默认关闭，只有确认不会删除稀疏有效回波后再启用。

输入与定位采用固定两线程执行器：输入回调持续加入累积器，后端按 `processing.rate_hz` 处理单一最新快照。没有定位任务队列，后端繁忙时旧待处理状态会被最新输入序号取代；累积点本身仍严格受上述三个上限约束。`processing.parallel_base_candidates` 可让开/关基地 ICP 并行，两个候选完成后才串行更新状态和连续帧门控。

连续帧门控不会靠旧快照自行“凑够帧数”：后端只处理包含新输入序号的快照，并要求最新输入帧至少含一个经过坐标变换后、XYZ 有限且通过当前距离/CropBox 的点。没有新输入时定时器不会重复定位。输入转换、TF、模型、场景点数等失败会重置基地/装甲板连续帧状态。

装甲板粗/细搜索候选总数受 `armor.max_search_candidates` 保护，极小步长或过大行程导致超限时拒绝搜索，而不会进行无界计算。成功结果还必须满足基地和装甲板各自的 `required_consecutive_frames`。

## 启动

主 launch 只启动定位节点，并可选 RViz，不强耦合任何 Livox 驱动。一般字符串参数为空时沿用 `params_file`；三个模型参数为空时自动使用 `dart_vision_lidar_model` 随包安装的 PCD。

用场地 YAML 启动：

```bash
ros2 launch dart_vision_lidar_bringup mid70_localization.launch.py \
  params_file:=/data/mid70/site_a.yaml use_rviz:=true
```

也可以直接覆盖所需入口参数：

```bash
ros2 launch dart_vision_lidar_bringup mid70_localization.launch.py \
  input_type:=pointcloud2 pointcloud2_topic:=/livox/lidar \
  output_frame:=map \
  base_open_pcd:=/data/mid70/base_open_fixed.pcd \
  base_closed_pcd:=/data/mid70/base_closed_fixed.pcd \
  armor_pcd:=/data/mid70/moving_armor.pcd \
  use_rviz:=true
```

`output_frame:=map` 要求系统中其他节点在输入时间戳提供 `map <- lidar_frame`；本 launch 不发布猜测外参。若修改 `lidar_frame`，还要在 RViz 修改 Fixed Frame 或提供相应 TF。

另有组合入口 `mid70_system.launch.py`。它总是 include 上述定位 launch，并可通过 `start_livox_driver` 选择是否同时 include 仓库现有旧版 Livox 驱动：`input_type:=pointcloud2` 对应 `livox_lidar_launch.py`，`input_type:=livox_custom` 对应 `livox_lidar_msg_launch.py`。默认不启动驱动，便于 rosbag 回放、driver2 或外部驱动接入：

```bash
# rosbag / 外部驱动：只启动定位和可选 RViz
ros2 launch dart_vision_lidar_bringup mid70_system.launch.py \
  params_file:=/data/mid70/site_a.yaml input_type:=pointcloud2 \
  start_livox_driver:=false use_rviz:=true

# 仓库旧版驱动 + PointCloud2 + 定位
ros2 launch dart_vision_lidar_bringup mid70_system.launch.py \
  params_file:=/data/mid70/site_a.yaml input_type:=pointcloud2 \
  start_livox_driver:=true use_rviz:=true

# 仓库旧版驱动 CustomMsg + 定位
ros2 launch dart_vision_lidar_bringup mid70_system.launch.py \
  params_file:=/data/mid70/site_a.yaml input_type:=livox_custom \
  start_livox_driver:=true use_rviz:=true
```

组合入口也绝不会运行 `dart_vision_model_builder`；模型构建只能由前述显式命令或 `build_models.launch.py` 触发。旧驱动的广播码、设备 JSON 和频率仍在其自身配置中维护。

### PointCloud2

本工作区实际 vendoring 的是旧版 `third_party/livox_ros2_driver`，不是 `livox_ros2_driver2`。驱动与定位分别启动：

```bash
# 终端 1：先按现场广播码和 JSON 配置旧驱动
ros2 launch livox_ros2_driver livox_lidar_launch.py

# 终端 2
ros2 launch dart_vision_lidar_bringup mid70_localization.launch.py \
  input_type:=pointcloud2 pointcloud2_topic:=/livox/lidar \
  params_file:=/data/mid70/site_a.yaml
```

PointCloud2 至少要有标量 FLOAT32/FLOAT64 `x/y/z`，`intensity` 可选。适配层校验字段布局、行步长和大小端。以后接 `livox_ros2_driver2` 时，推荐让它发布标准 `sensor_msgs/msg/PointCloud2`，这是稳定的兼容边界。

### CustomMsg

当前自定义适配器只订阅工作区已有的 `livox_interfaces/msg/CustomMsg`：

```bash
# 终端 1
ros2 launch livox_ros2_driver livox_lidar_msg_launch.py

# 终端 2
ros2 launch dart_vision_lidar_bringup mid70_localization.launch.py \
  input_type:=livox_custom livox_custom_topic:=/livox/lidar \
  params_file:=/data/mid70/site_a.yaml
```

这不承诺与 driver2 私有 CustomMsg ABI/类型兼容；若 driver2 使用另一个消息包，应改用 PointCloud2 或另加显式桥接层。

## 输出、状态与 RViz

| 默认话题 | 类型 | frame / 内容 |
| --- | --- | --- |
| `mid70_localization` | `dart_vision_interfaces/msg/Mid70Localization` | 总有效性、状态、置信度、基地/装甲板指标、连续帧、绿灯坐标/距离 |
| `debug/accumulated` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，后端当前处理的多帧累积快照 |
| `debug/filtered` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，预处理云 |
| `debug/base_open_initial_template` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，严格按 YAML 初值放置的开模板 |
| `debug/base_closed_initial_template` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，严格按 YAML 初值放置的关模板 |
| `debug/armor_initial_template` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，按基地初值与装甲板零位放置的初始模板 |
| `debug/base_open_template` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，ICP 后的开模板候选，候选无效时也用于诊断 |
| `debug/base_closed_template` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，ICP 后的关模板候选，候选无效时也用于诊断 |
| `debug/armor_template` | `sensor_msgs/msg/PointCloud2` | `lidar_frame`，最佳一维装甲板候选 |
| `debug/base_pose` | `geometry_msgs/msg/PoseStamped` | `output_frame`，缺输出 TF 时诊断回退为 `lidar_frame` |
| `debug/armor_pose` | `geometry_msgs/msg/PoseStamped` | 同上 |
| `debug/markers` | `visualization_msgs/msg/MarkerArray` | Base/Armor Initial 原点、CropBox、运动轴、绿灯、装甲板方向和有效性文本 |

状态码：`0 OK`、`1 INPUT_INVALID`、`2 TF_UNAVAILABLE`、`3 MODEL_UNAVAILABLE`、`4 INSUFFICIENT_POINTS`、`5 BASE_UNRELIABLE`、`6 BASE_UNSTABLE`、`7 ARMOR_UNRELIABLE`、`8 ARMOR_UNSTABLE`、`9 OUTPUT_TF_UNAVAILABLE`。`base_state` 为 `0 UNKNOWN / 1 OPEN / 2 CLOSED`。

一条消息只有一个 `status_code`。进入完整配准路径后，当前诊断优先级固定为：`output TF` → `base` → `armor/green light`；因此输出 TF 与基地同时失败时，码 9 会遮蔽基地失败码。应同时查看 `valid`、`base_valid`、`armor_valid`、各自置信度/残差/内点率/覆盖率/连续帧和 `status_message`。早于配准的输入、输入 TF、当前帧、模型或点数失败会直接返回对应状态。

只有 `STATUS_OK` 时总 `valid=true`。消费方必须先检查 `valid`，不得使用失效消息中的 NaN、候选位姿或旧状态做控制。

`use_rviz:=true` 会载入八路点云和 MarkerArray：累积云灰色、过滤云白色、开/关 Base Initial 模板分别为蓝色和紫色、Armor Initial 为橙色，配准后的开/关候选分别为绿色和红色，装甲板搜索候选为黄色。MarkerArray 还显示 `BASE INITIAL ORIGIN`、`ARMOR INITIAL ORIGIN`、青色 CropBox 线框、运动轴、绿灯球、装甲板箭头和有效性文本。开/关基地共享同一个 `base` 坐标系，因此共享一个基地原点标注；装甲板初始原点由 `t_lidar_base_initial * t_base_armor_zero` 得到。

Initial 模板严格来自启动时读取的初值；绿色/红色模板是 ICP 候选，即使候选因 `translation_bound_exceeded` 等条件被拒绝，也可能已经贴近场景，不能用它们反推初值是否正确。

Initial 话题和 MarkerArray 使用 Reliable + Transient Local QoS，RViz 后启动仍可收到。三路 Initial 点云只在节点启动时发布一次；实物调初值时先关闭所有 Registered，只比较 `Filtered` 与相应 Initial。基地初值大致对齐后，再调整 `armor.zero_translation_m/rpy_rad` 使 Armor Initial 对准机械零位，最后打开 Registered 检查 ICP、一维搜索修正和结果指标。修改 YAML 后必须重启节点。

CropBox 在 `lidar_frame` 中轴对齐，边界直接对应 `preprocess.crop_box_min_m/max_m`；`crop_box_negative=true` 时线框仍表示被排除的盒内区域。常用诊断：

```bash
ros2 topic echo /mid70_localization
ros2 topic hz /debug/filtered
ros2 topic info --verbose /livox/lidar
ros2 run tf2_ros tf2_echo map livox_frame
```

## 实物标定与上线

1. 固定雷达，核对输出轴、米制、时间戳、`frame_id`、频率和 QoS；录制可复现 rosbag。
2. 从 CAD 明确拆分三个网格，统一原点/方向后分别采样；用已知尺寸检查 PCD 包围盒，排除毫米/米和镜像错误。
3. 用全站仪、标定板或人工对应点求 `t_lidar_base_initial`，记录方向；在基地开、关实物状态分别检查模板覆盖。
4. 标定 `t_base_armor_zero`、运动轴正方向和机械行程端点，在零位和两端核验 `armor_axis_position_m`。
5. 在 armor frame 测量绿灯中心偏移，核验雷达 XYZ、输出 frame XYZ 和卷尺距离。
6. 用空场、遮挡、状态切换、全行程、边界视角和干扰物 rosbag 统计指标，再调 ROI、体素、ICP、分差和时序阈值。
7. 验证启动、空帧、短时遮挡、时间戳倒退、输入/输出 TF 中断时都为 `valid=false`，恢复后必须重新累计连续有效帧。
8. 长时间运行检查 CPU、内存、累积上限和输出频率，再让下游只使用 `valid=true` 数据。

## 已知限制

- 当前实际依赖旧 `livox_ros2_driver/livox_interfaces`，不是 driver2；driver2 PointCloud2 可接入，私有 CustomMsg 尚未适配。
- 多帧累积没有逐点去畸变、雷达/平台运动补偿或点时间插值；明显运动时需缩短窗口。
- 第一阶段没有相机融合、目标语义复核、弹道解算或控制输出。
- 不提供已确认的真实开/关/装甲板模型；模型缺失是安全失败，`valid=false`。
- 装甲板假设相对基地的零位姿、轴向固定，机械松动/变形不会由在线六自由度漂移吸收。
- 示例 ROI、采样密度、残差、内点率、覆盖率、状态分差和时序阈值必须使用实物与 rosbag 重新标定。
