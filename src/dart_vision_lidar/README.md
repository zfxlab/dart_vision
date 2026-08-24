# dart_vision_lidar

本目录仅用于组织 LiDAR 相关 ROS 2 package，本身不是 package，不应包含
`package.xml` 或顶层 `CMakeLists.txt`。

| Package | 职责 |
| --- | --- |
| `dart_vision_lidar_model` | 读取普通 YAML，离线将多个 PLY/STL 网格采样并生成 PCD |
| `dart_vision_lidar_localization` | Mid-70 在线点云累积、预处理、基地/装甲板定位和结果发布 |
| `dart_vision_lidar_bringup` | 部署参数、launch、RViz 和现场使用文档 |

公共结果消息仍由同级的 `dart_vision_interfaces` 提供。完整构建、模型生成、启动、
状态语义和实物标定说明见 `dart_vision_lidar_bringup/README.md`。

```bash
colcon build --symlink-install --packages-up-to dart_vision_lidar_bringup
source install/setup.bash
ros2 launch dart_vision_lidar_bringup mid70_localization.launch.py
```

模型输入必须预先位于各自的目标模板坐标系；离线 builder 只按配置将 PLY/STL 顶点
缩放到米，不会额外旋转、平移或中心化。
