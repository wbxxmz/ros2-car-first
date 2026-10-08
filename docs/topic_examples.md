# 常用话题示例

地面速度控制：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {z: 0.0}}" -r 10
```

飞行速度控制：

```bash
ros2 topic pub /uav/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.1}, angular: {z: 0.0}}" -r 10
```

打开和关闭 UAV 下方激光：

```bash
ros2 topic pub --once /uav/laser_fire std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /uav/laser_fire std_msgs/msg/Bool "{data: false}"
```
