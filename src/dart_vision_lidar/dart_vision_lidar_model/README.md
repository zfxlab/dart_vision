# dart_vision_lidar_model

Mid-70 LiDAR 模板的离线生成包。它读取普通 YAML 文件，将任意数量的 PLY 或 STL
三角网格按面积加权均匀采样、换算为米、可选 VoxelGrid 后保存为 binary PCD。
在线定位不依赖 yaml-cpp，也不会在启动时自动重建模型。

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select dart_vision_lidar_model --symlink-install
source install/setup.bash
```

## 批量生成

复制并修改
[`config/model_build_example.yaml`](config/model_build_example.yaml)，确认每个模型的
输入路径、单位和模板坐标系后运行：

```bash
ros2 run dart_vision_lidar_model dart_vision_model_builder \
  --config /absolute/path/mid70_models.yaml
```

这是普通 yaml-cpp 配置，不是 ROS 参数文件。`input_mesh` 和 `output_pcd` 的相对路径
均相对于 YAML 所在目录解析。顶层格式为：

```yaml
schema_version: 2
models:
  - name: moving_armor
    input_mesh: raw/armor.stl
    output_pcd: processed/armor.pcd
    scale_to_m: 0.001
    sample_count: 60000
    random_seed: 7003
    voxel_leaf_m: 0.003
    overwrite: false
```

PLY 和 STL 的顶点都必须已经位于该模型的目标模板坐标系。builder 不旋转、不平移、
不中心化，只按下式换算单位：

```text
p_pcd_m = scale_to_m * p_mesh
```

`scale_to_m` 用于将 CAD 数值换算为米，例如毫米模型填写 `0.001`，米制模型填写
`1.0`。`voxel_leaf_m: 0` 关闭降采样。模型名和输出路径必须唯一；输入必须是含有
可采样三角面的 `.ply` 或 `.stl` 普通文件，输出必须是 `.pcd`。输出目录会自动创建，
已有输出只有在 `overwrite: true` 时才会覆盖。任意校验、读取、采样或保存失败都会使
进程返回非零。
