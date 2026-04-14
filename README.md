# CVRP Planner for Isaac Sim + Nav2

一个面向 `ROS 2 Humble` 的多机器人任务分配与闭环导航示例项目。

它把这几件事接到了一起：

- `Isaac Sim` 仓库场景与多机器人
- `Qt` 任务点选择界面
- `CVRP` 任务分配
- `Nav2 FollowWaypoints` 闭环执行
- `RViz2` 路径与地图调试

## 项目结构

- `cvrp_planner/`
  ROS 2 Python 包源码
- `launch/`
  规划器、多机器人 Nav2、闭环启动文件
- `scripts/isaac/`
  Isaac Sim 场景生成与打开脚本
- `rviz/`
  RViz2 配置
- `cvrp_planner/maps/`
  当前默认占据地图
- `cvrp_planner/params/`
  默认参数与 Nav2 参数

## 当前默认流程

1. 生成 Isaac 多机器人导航场景：

```bash
bash scripts/isaac/run_full_warehouse_nav_scene.sh
```

2. 打开 Isaac 并启用 ROS 2 Bridge：

```bash
bash scripts/isaac/launch_full_warehouse_nav_ros.sh
```

3. 启动多机器人 Nav2：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cvrp_planner isaac_multi_nav.launch.py
```

4. 启动任务点界面与分配器：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cvrp_planner cvrp_planner.launch.py
```

5. 如果想一条命令直接跑导航和任务分配：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch cvrp_planner isaac_closed_loop.launch.py
```

## 默认地图与示例

- 默认地图：
  `cvrp_planner/maps/manual_map_from_MAP.yaml`
- 默认机器人配置：
  `scripts/isaac/full_warehouse_nav_robots.example.json`

## 开发

构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select cvrp_planner --symlink-install
```

测试：

```bash
source /opt/ros/humble/setup.bash
colcon test --packages-select cvrp_planner
```

## 备注

- 运行产物如 `build/`、`install/`、`log/`、策略图、结果 XML 和 Isaac 临时生成文件默认已忽略。
- 许可证：`MIT`，见 `LICENSE`。
