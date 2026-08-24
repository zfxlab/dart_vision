# dart_vision_lidar_localization

Mid-70 基地、移动装甲板与绿灯点云定位的 ROS 2 C++ 在线算法包。

本包接收 `sensor_msgs/msg/PointCloud2` 或工作区现有的
`livox_interfaces/msg/CustomMsg`，完成米制点云转换、多帧累积、预处理、基地
开关状态与位姿估计、移动装甲板沿机械轴的一维定位，以及绿灯坐标和距离计算。
定位结果同时包含有效性、置信度、状态码、位姿和 RViz 调试话题。
现有 MarkerArray 还会发布 `lidar_frame` 中轴对齐的青色 CropBox 线框，便于核对
`preprocess.crop_box_min_m/max_m` 是否完整覆盖目标。

节点使用有界的前后端结构：输入线程持续接收并累积雷达帧，定位后端按
`processing.rate_hz` 处理最新累积快照。两者之间没有任务队列，也不会保存等待处理的
历史快照；后端繁忙时，新帧继续进入受时间、帧数和点数三重限制的累积器。打开与关闭
基地候选可受控并行，但同一时刻最多运行一个完整定位周期。

离线 PLY/STL 采样与 PCD 生成属于 `dart_vision_lidar_model`；输入网格需预先位于目标
模板坐标系，builder 只将单位缩放到米。launch、部署 YAML 和
RViz 配置属于 `dart_vision_lidar_bringup`。在线节点不会自动生成或覆盖模型文件。

## 组成

- `dart_vision_lidar_localization_core`：点云累积、预处理、配准指标、基地定位、
  装甲板一维搜索和连续帧门控。
- `dart_vision_lidar_localization_adapters`：将 PointCloud2 和 Livox CustomMsg
  转换为统一的 `pcl::PointCloud<pcl::PointXYZI>`。
- `mid70_localization_node`：ROS 订阅、TF 查询、参数管理、结果与调试话题发布。

头文件与实现按相同结构分为四组：

```text
core/        点云基础类型、坐标变换、配准指标和时序门控
pipeline/    多帧点云累积与预处理
localizers/  基地定位和装甲板一维定位
ros/         ROS 消息适配、节点编排与程序入口
```

依赖方向为 `ros -> localizers/pipeline/core`、`localizers -> core`、
`pipeline -> core`，底层模块不反向依赖 ROS 节点。

所有内部长度统一为米。变换统一记为 `t_target_source`：

```text
p_target = t_target_source * p_source
```

输入点云先依据消息时间戳从 `header.frame_id` 变换到 `lidar_frame`，再进入累积和
定位流程。定位完成后才通过 `t_output_lidar` 转换到 `output_frame`。调试点云和结果按
后端处理频率发布，但累积计数对应被处理快照中的真实雷达帧数。

## 初值模板与配准候选

初值模板与在线候选使用含义不同的 RViz 调试话题：

- `debug/base_open_initial_template` 和 `debug/base_closed_initial_template` 严格使用
  YAML 的 `base.initial_translation_m`、`base.initial_rpy_rad`，表示
  `t_lidar_base_initial`。它们在节点启动时发布一次，并使用 Transient Local QoS，
  因此后启动的 RViz 也能看到。
- `debug/base_open_template` 和 `debug/base_closed_template` 是本周期 ICP 得到的开、关
  候选。即使候选最终因平移边界、旋转边界或质量指标被拒绝，仍会发布供诊断，不能把
  它们的位置误认为 YAML 初值。
- `debug/armor_initial_template` 严格使用
  `t_lidar_armor_initial = t_lidar_base_initial * t_base_armor_zero`，对应装甲板轴位置为零的
  初始模板；`debug/armor_template` 是一维粗/细搜索后的动态候选。

`debug/markers` 中的 `BASE INITIAL ORIGIN` 是开/关基地模板共享的 `base` 原点，
`ARMOR INITIAL ORIGIN` 是装甲板零位的 `armor` 原点。两者均固定使用 YAML 初值，不跟随
ICP 或一维搜索移动。

调整基地初值时，应先只显示 `Filtered` 和对应的 `Base ... Initial`，让未配准模板与场景
大致重合；然后再打开 `Base ... Registered`，观察 ICP 修正量和最终指标。修改 YAML 后需
重启节点，Initial 话题不会在运行期间随文件内容变化。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select dart_vision_lidar_localization
source install/setup.bash
```

## 直接运行

通常应通过 `dart_vision_lidar_bringup` 启动。需要单独调试节点时可运行：

```bash
ros2 run dart_vision_lidar_localization mid70_localization_node --ros-args \
  -p input_type:=pointcloud2 \
  -p pointcloud2_topic:=/livox/lidar \
  -p lidar_frame:=livox_frame \
  -p output_frame:=livox_frame \
  -p processing.rate_hz:=3.0 \
  -p model.base_open_pcd:=/data/mid70/base_open_fixed.pcd \
  -p model.base_closed_pcd:=/data/mid70/base_closed_fixed.pcd \
  -p model.armor_pcd:=/data/mid70/moving_armor.pcd
```

使用当前工作区的 Livox CustomMsg 时，将 `input_type` 设置为 `livox_custom`，并
配置 `livox_custom_topic`。如果使用其他版本 Livox 驱动的私有 CustomMsg，建议
优先改用标准 PointCloud2，或提供独立适配层。

## 可靠性约束

- 基地配准不会只凭 `hasConverged()` 判定成功，还会检查局部位姿边界、截断最近邻
  残差、内点率、模板覆盖率、开关状态分差和连续帧状态。
- 基地 ICP 只优化 initial base frame 中的 `x/y/yaw`，Z 平移及 Z 轴方向保持 YAML 初值；
  `base.max_rotation_delta_rad` 在该约束下表示最大 yaw 修正量。
- 装甲板只允许沿配置的机械轴进行一维粗搜索和细搜索，不进行在线自由六自由度漂移。
- 完整 CAD 与不完整场景采用单向质量评估，不强制双向全量匹配。
- 模型缺失、输入或 TF 异常、当前帧无有效新点、质量指标不足或连续帧不足时，均发布
  `valid=false`。
- 下游必须先检查 `valid`，不得使用无效消息中的候选位姿或坐标进行控制。
- 后端只处理不同输入序号对应的新快照，不会重复使用旧点云推进连续帧门控。

完整参数、状态码、话题、RViz 配置和实物标定步骤见
`dart_vision_lidar_bringup/README.md`。
