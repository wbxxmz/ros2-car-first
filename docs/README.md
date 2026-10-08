# iCAN 任务一巡检仿真用户工作区

这个工作区面向参赛者和用户，提供 iCAN 任务一“城市应急全域巡检”的基础仿真环境、地面 Nav2 配置和最小用户程序模板。

当前用户版暂不包含裁判系统。裁判系统源码计划在比赛开始前单独发放。

## 已提供内容

| 内容 | 说明 |
| --- | --- |
| `packages/ican_inspection_env_binary.tar.gz` | 预编译仿真环境和地面 Nav2 配置包，包含 Gazebo 插件、世界、地图、launch 和 RViz 配置。 |
| `src/ican_inspection_examples` | 用户示例源码包，演示如何订阅里程计、调用 Nav2、发布 `/cmd_vel` 和 `/uav/cmd_vel`。 |

## 快速开始

```bash
cd <ican_inspection_user_ws>
bash scripts/00_install_ros2_gazebo.sh
bash scripts/01_setup_inspection_workspace.sh
source scripts/setup_env.sh
bash scripts/check_sim_env.sh
bash scripts/run_inspection_env.sh
```

`run_inspection_env.sh` 会打开两个终端标签：

- `inspection sim`：Gazebo 仿真场景
- `ground nav2`：地面 Nav2 和 RViz

## 运行最小模板

另开终端：

```bash
cd <ican_inspection_user_ws>
source scripts/setup_env.sh
ros2 run ican_inspection_examples minimal_user_node
```

模板会直接执行一次 `/uav/cmd_vel` 开环速度控制演示：上升、前进、悬停后下降。

## 用户需要实现

- 地面阶段：可以直接发布 `/cmd_vel`，也可以调用 Nav2 `navigate_to_pose` action 到达指定坐标。
- 飞行阶段：发布 `/uav/cmd_vel` 控制上升、下降、前后左右移动和偏航；需要闭环控制时可再订阅 `/uav/odom` 自行实现。
- 识别阶段：订阅相机、雷达或点云话题，实现自己的窗口目标和任务目标识别。

更多接口见 [仿真环境与接口说明.md](docs/仿真环境与接口说明.md)。
