# stl_to_pcd

一个独立的离线 STL 转 PCD 工具。它按三角形面积对 STL 表面进行均匀随机采样，
可选体素降采样，最后写出 binary PCD。

## 构建

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select stl_to_pcd --symlink-install
source install/setup.bash
```

## 使用

```bash
ros2 run stl_to_pcd stl_to_pcd input.stl output.pcd \
  --samples 100000 \
  --scale 0.001 \
  --voxel 0.005 \
  --seed 42
```

- `--samples`：体素处理前的表面候选点数，默认 `100000`。
- `--scale`：输入坐标缩放倍数，默认 `1.0`。例如 STL 使用毫米、PCD 需要米时传
  `0.001`。
- `--voxel`：体素边长，单位为缩放后的坐标单位。默认 `0`，即不降采样。
- `--seed`：随机种子，相同输入和参数可以得到相同采样结果，默认 `0`。
- `--overwrite`：允许覆盖已有输出；默认拒绝覆盖。

开启体素降采样后，最终输出点数通常小于 `--samples`。运行
`ros2 run stl_to_pcd stl_to_pcd --help` 可以查看完整参数说明。
