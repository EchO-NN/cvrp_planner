# Isaac Sim Scripts

这个目录只保留当前闭环流程真正用到的 Isaac Sim 脚本。

## 保留的脚本

- `build_full_warehouse_nav_scene.py`
  基于 Isaac 官方 `full_warehouse.usd` 生成多机器人导航场景。
- `full_warehouse_nav_robots.example.json`
  机器人初始位姿和传感器开关示例配置。
- `run_full_warehouse_nav_scene.sh`
  一键生成导航场景。
- `open_warehouse_scene.py`
  用 Isaac Python 打开场景，并在运行时补齐 `odom` / `cmd_vel` 图。
- `open_stage_clean.sh`
  用干净环境打开场景，仅观察。
- `open_stage_with_ros.sh`
  用带 ROS 2 Bridge 的环境打开场景。
- `launch_full_warehouse_nav_ros.sh`
  生成并直接打开最终导航场景。
- `export_scene_occupancy_map.py`
  从 Isaac 场景导出 ROS 占据地图。
- `run_export_scene_occupancy_map.sh`
  一键导出占据地图。

## 推荐流程

1. 生成导航场景：

```bash
bash scripts/isaac/run_full_warehouse_nav_scene.sh
```

2. 打开 Isaac 场景并启用 ROS 2 Bridge：

```bash
bash scripts/isaac/launch_full_warehouse_nav_ros.sh
```

3. 如需重新导出占据地图：

```bash
bash scripts/isaac/run_export_scene_occupancy_map.sh
```

## 说明

- 所有运行期输出都写到 `scripts/isaac/generated/`，这个目录默认不纳入版本管理。
- 默认导航场景配置见 `full_warehouse_nav_robots.example.json`。
- 如果 Isaac 安装路径不是默认值，请先设置：

```bash
export ISAAC_SIM_ROOT=/path/to/isaacsim/_build/linux-x86_64/release
```
