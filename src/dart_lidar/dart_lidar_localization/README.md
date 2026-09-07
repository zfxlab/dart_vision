# dart_lidar_localization

统一完成基地配准和基地上方移动检测模块定位。点云预处理与累积仍由独立包负责，本包接收一批已经累积完成的
点云，先以 description 发布的场地 TF 为基地初值执行 NDT（可选）和 GICP，再把整批点云变换到实际
`base_link`，最后沿 `dart_detection_module_slide_joint` 的 X 轴做一维粗到细搜索。

## 目录

- `base/`：基地模型的多分辨率 NDT/GICP 和质量验收。
- `module/`：滑轨 ROI、一维位置搜索、技术质量验收与可选时间连续性检查。
- `pipeline/`：固定“基地后模块”的顺序与坐标变换约定。
- `io/`：PCD/PLY 模型加载和带时间戳的结果 YAML。
- `ros/`：TF 快照、点云订阅与统一观测发布。

节点发布最小接口 `dart_interfaces/msg/LidarObservation`。详细 RMSE、重叠率、对应点数和迭代次数只用于日志与
可选 YAML 记录，不再占用在线消息。模块关节状态由下游 `dart_target_estimation` 根据观测结果统一发布。

离线配置还发布两类调试点云：`/lidar/offline/aligned` 只包含通过质量验收的结果；
`/lidar/offline/candidate_aligned` 包含已完成 NDT/GICP、但可能被重叠率、对应点数等质量规则拒绝的候选结果。
两者均位于 `base_link` 坐标系，使用 Best Effort QoS。

在线和离线配置分别位于 `dart_bringup/config/lidar/{online,offline}/localization.yaml`。两份配置默认都要求
填写基地模型和移动模块模型路径；在模型未准备好时保持 launch 参数 `start_localization:=false`。
