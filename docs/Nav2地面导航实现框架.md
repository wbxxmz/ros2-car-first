# Nav2 自动地面导航实现框架（逐行详解版）

本文档用大白话逐行解释 Nav2 地面导航的实现代码，目标是让你看懂每一行在干什么，以及为什么这么写。

---

## 1. 整体架构（先搞清楚谁在干什么）

导航涉及三个进程，全部由 `scripts/run_inspection_env.sh` 启动：

| 进程 | 启动脚本/Launch | 作用 |
| --- | --- | --- |
| Gazebo 仿真 | `ros2 launch ican_inspection_sim inspection.launch.py` | 提供物理场景、机器人、传感器（`/odom`、`/scan`） |
| Nav2 导航栈 | `ros2 launch ican_inspection_navigation nav2_ground.launch.py` | 地图、定位、规划、控制，输出 `/cmd_vel` |
| 用户节点 | `ros2 run ican_inspection_examples minimal_user_node` | 调用 Nav2 的 action 接口下达目标点 |

数据流：

```
你的节点 ──"去(1.125, -1.35)"──▶ Nav2（规划+控制）──/cmd_vel──▶ 机器人动起来
                                            ▲                          │
                                            │                          ▼
                                       /scan, /odom ◀──────── 传感器回报位置
```

你只需要做一件事：**告诉 Nav2 目标点坐标**，剩下的路径规划、避障、速度控制全是 Nav2 的事。

---

## 2. 核心代码逐行解释

下面是你模板 `minimal_user_node.py` 里已经有的 `send_ground_nav_goal()` 方法。我把每一行都拆开讲。

### 2.1 导入部分

```python
import math                                    # 数学函数库，用来算四元数
from action_msgs.msg import GoalStatus         # action 的状态类型（成功/失败等）
from geometry_msgs.msg import PoseStamped, Twist  # 位姿消息、速度消息
from nav2_msgs.action import NavigateToPose    # Nav2 导航的 action 类型
from nav_msgs.msg import Odometry              # 里程计消息类型
import rclpy                                   # ROS 2 的 Python 客户端库
from rclpy.action import ActionClient          # action 客户端（用来调用 Nav2）
from rclpy.node import Node                    # ROS 2 节点基类
from std_msgs.msg import Bool                  # 布尔消息类型（激光开关用）
```

每一行的作用：

| 行 | 作用 | 类比 |
|----|------|------|
| `import math` | 引入数学函数 | 拿出计算器 |
| `from action_msgs.msg import GoalStatus` | 引入 action 状态常量 | 拿出"成功/失败"的判定标准 |
| `from geometry_msgs.msg import PoseStamped, Twist` | 引入位姿和速度消息 | 拿出"位置"和"速度"的信封格式 |
| `from nav2_msgs.action import NavigateToPose` | 引入 Nav2 导航 action 的定义 | 拿出"导航下单"的订单格式 |
| `from nav_msgs.msg import Odometry` | 引入里程计消息 | 拿出"位置回报"的信封格式 |
| `import rclpy` | 引入 ROS 2 核心库 | 拿出 ROS 2 的工具箱 |
| `from rclpy.action import ActionClient` | 引入 action 客户端 | 拿出"下订单"的工具 |
| `from rclpy.node import Node` | 引入节点基类 | 拿出"开一个节点"的模板 |
| `from std_msgs.msg import Bool` | 引入布尔消息 | 拿出"开关"的信封格式 |

### 2.2 工具函数：偏航角转四元数

```python
def quaternion_from_yaw(yaw: float):
    """偏航角转四元数（只绕 z 轴旋转）。"""
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)
```

逐行解释：

| 行 | 代码 | 解释 |
|----|------|------|
| 1 | `def quaternion_from_yaw(yaw: float):` | 定义一个函数，输入是偏航角 `yaw`（弧度），输出是四元数 |
| 2 | `half = 0.5 * yaw` | 把角度除以 2。四元数的公式里要用到半角，所以先算出来 |
| 3 | `return 0.0, 0.0, math.sin(half), math.cos(half)` | 返回四元数 `(x, y, z, w)`。因为只绕 z 轴旋转，所以 x=0, y=0，z=sin(θ/2)，w=cos(θ/2) |

**为什么需要这个？**
ROS 里表示方向用**四元数**（四个数 x,y,z,w），而我们习惯用**偏航角**（一个数，绕 z 轴转多少度）。这个函数就是把"一个数的角度"转成"四个数的四元数"。

> 类比：你说"向右转 90 度"，机器人听不懂，它需要的是一个四元数格式。这个函数就是翻译官。

### 2.3 类初始化：创建 action 客户端

```python
class MinimalUserNode(Node):
    def __init__(self) -> None:
        super().__init__("minimal_user_node")
        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action_name").value)
        )
```

逐行解释：

| 行 | 代码 | 解释 |
|----|------|------|
| 1 | `class MinimalUserNode(Node):` | 定义你的节点类，继承 ROS 2 的 `Node` 基类。继承就是"我是一个 ROS 节点，拥有节点的所有能力" |
| 2 | `def __init__(self) -> None:` | 构造函数，节点创建时自动执行 |
| 3 | `super().__init__("minimal_user_node")` | 调用父类（Node）的构造函数，给节点起名叫 `minimal_user_node` |
| 4-6 | `self.nav_client = ActionClient(...)` | 创建一个 action 客户端，相当于"打开导航 App，准备下单" |
| | `self` | 这个客户端属于当前节点 |
| | `NavigateToPose` | 用的 action 类型（导航专用） |
| | `"navigate_to_pose"` | action 的名字，要和 Nav2 提供的一致 |

**`ActionClient` 是什么？**
就像外卖 App 的客户端。你（客户端）下单 → 商家（Nav2 的 action server）接单 → 配送 → 告诉你结果。

### 2.4 核心方法：发送导航目标

这是整个地面导航最重要的方法：

```python
def send_ground_nav_goal(self, waypoint) -> bool:
    # 第 1 步：等待 Nav2 就绪
    if not self.nav_client.wait_for_server(timeout_sec=10.0):
        self.get_logger().error("Nav2 navigate_to_pose action server 未就绪。")
        return False

    # 第 2 步：拆开目标点坐标
    x, y, yaw = waypoint

    # 第 3 步：把偏航角转成四元数
    qx, qy, qz, qw = quaternion_from_yaw(yaw)

    # 第 4 步：构造目标位姿消息
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = self.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw

    # 第 5 步：把位姿装进导航目标
    goal = NavigateToPose.Goal()
    goal.pose = pose

    # 第 6 步：发送目标，异步等待接单
    send_future = self.nav_client.send_goal_async(goal)
    if not self.wait_for_future(send_future):
        return False
    goal_handle = send_future.result()

    # 第 7 步：检查 Nav2 是否接单
    if not goal_handle or not goal_handle.accepted:
        return False

    # 第 8 步：等待导航执行结果
    result_future = goal_handle.get_result_async()
    if not self.wait_for_future(result_future):
        return False

    # 第 9 步：判断是否成功
    result = result_future.result()
    if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
        return False

    # 第 10 步：Nav2 到达后做闭环精修（确保精准进入任务点）
    self.refine_ground_pose(x, y, yaw)
    return True
```

下面每一步详细讲：

#### 第 1 步：等待 Nav2 就绪

```python
if not self.nav_client.wait_for_server(timeout_sec=10.0):
    self.get_logger().error("Nav2 navigate_to_pose action server 未就绪。")
    return False
```

- `wait_for_server(timeout_sec=10.0)`：等 Nav2 的 action server 启动，最多等 10 秒
- 如果 10 秒内没等到（返回 False），打印错误并返回失败
- **为什么需要？** Nav2 启动需要时间（加载地图、初始化定位等），你不能一上来就发目标，得等它准备好

> 类比：你打开外卖 App，得等商家上线才能下单。`wait_for_server` 就是"等商家上线"。

#### 第 2 步：拆开目标点坐标

```python
x, y, yaw = waypoint
```

- `waypoint` 是一个元组 `(x, y, yaw)`，比如 `(1.125, -1.350, 0.0)`
- 这行把它拆成三个变量：`x=1.125`，`y=-1.350`，`yaw=0.0`

#### 第 3 步：偏航角转四元数

```python
qx, qy, qz, qw = quaternion_from_yaw(yaw)
```

- 调用前面讲的工具函数，把 `yaw`（角度）转成四元数 `(qx, qy, qz, qw)`
- 因为只绕 z 轴，`qx=0`，`qy=0`

#### 第 4 步：构造目标位姿消息

```python
pose = PoseStamped()
```
创建一个位姿消息对象。`PoseStamped` = 位姿 + 时间戳 + 坐标系。

```python
pose.header.frame_id = "map"
```
- `header` 是消息的头部，存元信息
- `frame_id = "map"` 告诉 Nav2：这个坐标是在**地图坐标系**下的
- **为什么是 map？** 因为目标点是地图上的固定位置，不是机器人自己的相对位置

```python
pose.header.stamp = self.get_clock().now().to_msg()
```
- 打上当前时间戳
- `self.get_clock().now()` 获取当前时间，`.to_msg()` 转成消息格式

```python
pose.pose.position.x = x
pose.pose.position.y = y
```
- 设置目标位置的 x、y 坐标
- `position` 是位置（x, y, z），地面导航只用到 x 和 y，z 默认为 0

```python
pose.pose.orientation.x = qx
pose.pose.orientation.y = qy
pose.pose.orientation.z = qz
pose.pose.orientation.w = qw
```
- 设置目标朝向（四元数）
- `orientation` 是朝向，用四元数表示

#### 第 5 步：把位姿装进导航目标

```python
goal = NavigateToPose.Goal()
goal.pose = pose
```
- `NavigateToPose.Goal()` 是导航 action 的"订单"格式
- 把刚才构造好的位姿 `pose` 填进订单的 `pose` 字段

#### 第 6 步：发送目标

```python
send_future = self.nav_client.send_goal_async(goal)
```
- `send_goal_async(goal)` 异步发送目标
- "异步"的意思是：发出去后不阻塞，立即返回一个 `future` 对象
- `future` 就像一张"取餐号"，你拿着号，等叫号

```python
if not self.wait_for_future(send_future):
    return False
```
- `wait_for_future` 会循环处理 ROS 消息，直到 future 有结果
- 如果没等到（节点被关闭等），返回失败

```python
goal_handle = send_future.result()
```
- 从 future 里取出结果，叫 `goal_handle`
- `goal_handle` 是 Nav2 给你的"订单号"，用它可以查导航进度

#### 第 7 步：检查 Nav2 是否接单

```python
if not goal_handle or not goal_handle.accepted:
    return False
```
- `goal_handle` 为空 → 没拿到订单号
- `goal_handle.accepted` 为 False → Nav2 拒绝了你的目标（可能目标点无效）
- 任一条件满足都返回失败

#### 第 8 步：等待导航执行结果

```python
result_future = goal_handle.get_result_async()
```
- 通过订单号 `goal_handle` 申请获取最终结果
- 又得到一个 future，这次等的是"导航完成"的通知

```python
if not self.wait_for_future(result_future):
    return False
```
- 继续等，直到导航完成

#### 第 9 步：判断是否成功

```python
result = result_future.result()
return result is not None and result.status == GoalStatus.STATUS_SUCCEEDED
```
- `result` 是导航结果，里面有个 `status` 字段
- `GoalStatus.STATUS_SUCCEEDED` 是"成功"状态
- 如果结果不为空且状态是成功，返回 `True`；否则返回 `False`

**`GoalStatus` 的可能值：**

| 状态 | 含义 |
|------|------|
| `STATUS_SUCCEEDED` | 导航成功，到达目标 |
| `STATUS_CANCELED` | 导航被取消 |
| `STATUS_ABORTED` | 导航中止（失败） |

### 2.5 等待 future 的辅助方法

```python
def wait_for_future(self, future) -> bool:
    while rclpy.ok() and not future.done():
        rclpy.spin_once(self, timeout_sec=0.1)
    return future.done()
```

逐行解释：

| 行 | 代码 | 解释 |
|----|------|------|
| 1 | `while rclpy.ok() and not future.done():` | 只要 ROS 没被关闭，且 future 还没结果，就循环 |
| 2 | `rclpy.spin_once(self, timeout_sec=0.1)` | 处理一次 ROS 消息（最多等 0.1 秒）。**必须调用**，否则收不到 Nav2 的回复 |
| 3 | `return future.done()` | 返回 future 是否完成（True 表示等到了结果） |

**为什么需要 `spin_once`？**
ROS 2 的消息是异步的，你必须主动"转一下"节点来处理收到的消息。`spin_once` 就是"转一下，看看有没有新消息"。如果不调它，`send_goal_async` 发出去的目标永远收不到回复。

> 类比：你点了外卖，得时不时看一下手机有没有"商家接单""骑手送达"的通知。`spin_once` 就是"看一下手机"。

---

## 3. 在 `run()` 里调用：遍历所有巡检点

光有 `send_ground_nav_goal` 还不够，你需要在 `run()` 里循环调用它，让机器人依次到达每个巡检点：

```python
def run(self) -> None:
    self.get_logger().info("开始地面巡检")

    for name, point in self.ground_task_waypoints.items():
        self.get_logger().info(f"导航到 {name}: {point}")
        if self.send_ground_nav_goal(point):
            self.get_logger().info(f"已到达 {name}")
        else:
            self.get_logger().warn(f"未到达 {name}，跳过")
        time.sleep(0.5)

    self.stop_ground()
    self.get_logger().info("地面巡检完成")
```

逐行解释：

| 行 | 代码 | 解释 |
|----|------|------|
| 1 | `def run(self) -> None:` | 主流程方法，节点启动后执行 |
| 2 | `self.get_logger().info("开始地面巡检")` | 打印日志：开始巡检 |
| 4 | `for name, point in self.ground_task_waypoints.items():` | 遍历 5 个巡检点。`name` 是名字（如 "ground_0"），`point` 是坐标 `(x, y, yaw)` |
| 5 | `self.get_logger().info(f"导航到 {name}: {point}")` | 打印当前要去哪个点 |
| 6 | `if self.send_ground_nav_goal(point):` | 调用导航方法，让机器人去这个点。返回 True 表示到达 |
| 7 | `self.get_logger().info(f"已到达 {name}")` | 到达了，打印成功 |
| 9 | `self.get_logger().warn(f"未到达 {name}，跳过")` | 没到达，打印警告，继续下一个 |
| 10 | `time.sleep(0.5)` | 到达后停 0.5 秒，缓冲一下 |
| 13 | `self.stop_ground()` | 全部巡检完，停车 |
| 14 | `self.get_logger().info("地面巡检完成")` | 打印完成 |

**`self.ground_task_waypoints` 是什么？**
在 `__init__` 里定义的字典：

```python
self.ground_task_waypoints = {
    "ground_0": (1.125, -1.350, 0.0),        # 方块55x40，长边x，车头朝x
    "ground_1": (0.000, -1.350, 0.0),        # 方块55x40，长边x，车头朝x
    "ground_2": (0.000, -2.700, 0.0),        # 方块55x40，长边x，车头朝x
    "ground_3": (1.125, -1.925, math.pi/2),  # 方块40x55，长边y，车头朝y
    "ground_4": (1.125, -0.775, math.pi/2),  # 方块40x55，长边y，车头朝y
    "ground_5": (0.000, 0.000, 0.0),        # 指挥中心55x40，长边x，车头朝x
}
```

每个值是 `(x, y, yaw)`——x 坐标、y 坐标、到达时的朝向角。

> **朝向必须和方块长边匹配！** 小车 footprint 约 44cm×36cm，任务点方块有两种朝向：55×40（长边沿 x）和 40×55（长边沿 y）。如果 yaw 设错，小车的 44cm 边会进不去方块的 40cm 边。详见下方"任务点朝向分析"章节。

---

## 4. 坐标系说明

| 坐标系 | frame_id | 说明 |
| --- | --- | --- |
| 地图 | `map` | 全局固定坐标系，目标点在此坐标系下给出 |
| 里程计 | `odom` | 局部连续坐标系，由底盘插件发布 |
| 机器人 | `base_footprint` | 机器人底盘中心 |

TF 链：`map → odom → base_footprint → laser_link`

- `map → odom`：由 AMCL 发布（定位修正）
- `odom → base_footprint`：由麦克纳姆轮插件发布（里程计推算）

目标点用 `map` 坐标系。当前 Nav2 配置中 `yaw_goal_tolerance: 0.10`（约 5.7°），即**到达时朝向也必须在 ±5.7° 以内才算成功**，不是只看 XY 到位。

---

## 5. 场地坐标参考

从 `inspection_task1.world` 提取的关键位置：

| 元素 | 坐标 (x, y) |
| --- | --- |
| 机器人起点 | (0.0, 0.0) |
| 指挥中心 | (0.0, 0.0) |
| 地面巡检点 0 | (1.125, -1.350) |
| 地面巡检点 1 | (0.000, -1.350) |
| 地面巡检点 2 | (0.000, -2.700) |
| 地面巡检点 3 | (1.125, -1.925) |
| 地面巡检点 4 | (1.125, -0.775) |
| 窗户 1 | (1.925, -0.328) |
| 窗户 2 | (1.925, -2.373) |
| 空中任务点 1 | (3.080, -0.320) |
| 空中任务点 2 | (3.080, -1.355) |

场地范围：x ∈ [-0.8, 4.25]，y ∈ [-3.375, 0.675]。

---

## 6. 运行步骤

1. 启动仿真和 Nav2（会打开 Gazebo + RViz）：
   ```bash
   cd /home/yjh/下载/inspection_ws
   source scripts/setup_env.sh
   bash scripts/run_inspection_env.sh
   ```

2. 等待 Nav2 完全启动（RViz 中地图显示、AMCL 粒子收敛），另开终端运行用户节点：
   ```bash
   source scripts/setup_env.sh
   ros2 run ican_inspection_examples minimal_user_node
   ```

3. 在 RViz 中观察全局路径（绿色）和机器人运动。

---

## 7. 调参要点（提高到点精度的完整方案）

Nav2 参数文件：`binary_install/share/ican_inspection_navigation/config/nav2_ground_params.yaml`

为了让小车**精准、朝向正确地进入任务点方块**，需要从三个层面调整：

### 7.1 目标检查器容差（最关键）

| 参数 | 原值 | 改后值 | 位置 | 作用 |
| --- | --- | --- | --- | --- |
| `xy_goal_tolerance` | 0.12 | **0.03** | general_goal_checker | 位置精度：±12cm → ±3cm |
| `yaw_goal_tolerance` | 3.14159 | **0.10** | general_goal_checker | 朝向精度：不检查 → ±5.7° |
| `xy_goal_tolerance` | 0.12 | **0.03** | FollowPath (DWB) | DWB 内部位置容差同步收紧 |
| `yaw_goal_tolerance` | （无） | **0.10** | FollowPath (DWB) | DWB 内部朝向容差同步收紧 |
| `tolerance` | 0.12 | **0.05** | planner_server.GridBased | 全局规划终点容差 |

> 注意：`general_goal_checker` 和 `FollowPath` 两处的容差必须**同步设置**，否则会出现"DWB 觉得够近停了，但目标检查器觉得没到"的卡死情况。

### 7.2 速度参数（保持原值，不建议降速）

> **比赛评分规则**：同分情况下，用时少的队伍排名靠前。降速会增加完成时间，**得不偿失**，所以速度参数建议保持原值。

| 参数 | 原值 | 建议 | 说明 |
| --- | --- | --- | --- |
| `max_vel_x` | 0.65 | **保持 0.65** | 最大前进速度 |
| `max_vel_y` | 0.65 | **保持 0.65** | 最大横向速度 |
| `max_vel_theta` | 0.90 | **保持 0.90** | 最大转向速度 |
| `max_speed_xy` | 0.75 | **保持 0.75** | 合速度上限 |
| `min_vel_x` | -0.45 | **保持 -0.45** | 反向速度 |
| `min_vel_y` | -0.45 | **保持 -0.45** | 反向速度 |

> 精度问题由"容差收紧 + 闭环精修"解决，不需要靠降速。降速只会让完成时间变长，影响评分。

### 7.3 精细停车判定

| 参数 | 原值 | 改后值 | 作用 |
| --- | --- | --- | --- |
| `trans_stopped_velocity` | 0.10 | **0.02** | 速度低于 2cm/s 才算停（原 10cm/s） |
| `short_circuit_trajectory_evaluation` | true | **false** | 评估所有轨迹选最优，不提前结束 |

> `trans_stopped_velocity` 是关键！原来小车还有 10cm/s 的速度时 DWB 就认为"停了"，结果小车还在滑行，自然停不准。

### 7.4 其他保持不变的参数

| 参数 | 值 | 说明 |
| --- | --- | --- |
| `robot_model_type` | `OmniMotionModel` | 小车是麦克纳姆轮，全向模型正确，**不要改成差速** |
| `max_vel_y / min_vel_y` | 非零 | 麦克纳姆轮需要横向速度，**不要设为 0** |
| `acc_lim_y / decel_lim_y` | 非零 | 横向加速度，麦克纳姆轮需要 |
| `inflation_radius` | 0.24 | 障碍物膨胀半径，避障裕度 |
| `initial_pose` | (0,0,0) | 初始位姿，与起点一致 |
| `acc_lim_*` | 1.8 | 加速度限制，影响加减速平滑度 |

---

## 8. 任务点朝向分析（yaw 怎么设）

从 `inspection_task1.world` 提取的任务点方块尺寸：

| 任务点 | 坐标 (x, y) | 方块尺寸 (x × y) | 长边方向 | 正确 yaw |
| --- | --- | --- | --- | --- |
| ground_0 | (1.125, -1.35) | 0.55 × 0.40 | x | **0.0** |
| ground_1 | (0.0, -1.35) | 0.55 × 0.40 | x | **0.0** |
| ground_2 | (0.0, -2.70) | 0.55 × 0.40 | x | **0.0** |
| ground_3 | (1.125, -1.925) | 0.40 × 0.55 | y | **π/2** |
| ground_4 | (1.125, -0.775) | 0.40 × 0.55 | y | **π/2** |
| ground_5 | (0.0, 0.0) | 0.55 × 0.40 | x | **0.0** |

小车 footprint：`[[0.22, 0.18], [0.22, -0.18], [-0.22, -0.18], [-0.22, 0.18]]`，即 **44cm(x) × 36cm(y)**。

**判断逻辑**：小车的 44cm 边必须对应方块的 55cm 边，36cm 边对应 40cm 边，才能完全进入。

- 方块 55×40（长边 x）：yaw=0，小车 44(x) 进 55(x)，36(y) 进 40(y) → ✅
- 方块 40×55（长边 y）：yaw=π/2，小车旋转 90° 后 36(x) 进 40(x)，44(y) 进 55(y) → ✅
- 如果 yaw 设错（如 ground_3 设 0）：小车 44(x) 进 40(x) → ❌ 进不去

> **比赛评分规则**：机器人到达目标点如未完全进入任务点内，裁判酌情给分。所以 yaw 必须设对，否则可能扣分。

---

## 9. 闭环精修（代码层面的最终保险）

仅靠 Nav2 调参可能还有几厘米的偏差。在 `send_ground_nav_goal()` 返回成功后，**你的代码接管 `/cmd_vel` 做最后的闭环精修**，能达到 ±2cm 的精度。

### 9.1 新增方法 `get_ground_pose()`

```python
def get_ground_pose(self):
    if self.ground_odom is None:
        return None
    p = self.ground_odom.pose.pose.position
    o = self.ground_odom.pose.pose.orientation
    yaw = self.yaw_from_quaternion(o.x, o.y, o.z, o.w)
    return p.x, p.y, yaw
```

从 `/odom` 订阅获取小车当前 `(x, y, yaw)`，和 `get_uav_pose()` 类似。

### 9.2 新增方法 `refine_ground_pose()`（两阶段精修）

```python
def refine_ground_pose(self, x, y, yaw,
                       pos_tol=0.03, yaw_tol=0.10,
                       max_xy_speed=0.08, max_yaw_speed=0.25,
                       timeout_sec=10.0) -> bool:
    deadline = time.monotonic() + timeout_sec
    kp_xy = 1.5
    kp_yaw = 1.5

    # ===== 阶段1：精修位置（只移动，不转朝向）=====
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(self, timeout_sec=0.0)
        pose = self.get_ground_pose()
        if pose is None:
            time.sleep(0.05)
            continue
        cx, cy, cyaw = pose
        err_x = x - cx
        err_y = y - cy
        if abs(err_x) < pos_tol and abs(err_y) < pos_tol:
            break                                    # 位置到位，进入阶段2
        body_x = err_x * math.cos(-cyaw) - err_y * math.sin(-cyaw)
        body_y = err_x * math.sin(-cyaw) + err_y * math.cos(-cyaw)
        cmd = Twist()
        cmd.linear.x = min(max_xy_speed, max(-max_xy_speed, body_x * kp_xy))
        cmd.linear.y = min(max_xy_speed, max(-max_xy_speed, body_y * kp_xy))
        # 注意：不发 angular.z，避免旋转导致位置漂移
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.03)

    # ===== 阶段2：精修朝向（只旋转，不移动位置）=====
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(self, timeout_sec=0.0)
        pose = self.get_ground_pose()
        if pose is None:
            time.sleep(0.05)
            continue
        cx, cy, cyaw = pose
        err_yaw = yaw - cyaw
        while err_yaw > math.pi:
            err_yaw -= 2 * math.pi
        while err_yaw < -math.pi:
            err_yaw += 2 * math.pi
        if abs(err_yaw) < yaw_tol:
            self.cmd_vel_pub.publish(Twist())        # 停住
            return True
        cmd = Twist()
        # 注意：不发 linear.x/y，避免移动导致朝向偏差
        cmd.angular.z = min(max_yaw_speed, max(-max_yaw_speed, err_yaw * kp_yaw))
        self.cmd_vel_pub.publish(cmd)
        time.sleep(0.03)

    self.cmd_vel_pub.publish(Twist())
    return False
```

**为什么要分两阶段？（重要！）**

如果同时控制位置和朝向，会出现**耦合振荡**：

```
位置到位 → 开始转朝向 → 旋转时麦克纳姆轮有轻微位置漂移 → 位置又偏了
→ 修位置 → 朝向又变了 → 死循环，小车在目标点附近反复转很久
```

分两阶段后，位置和朝向**解耦**，互不干扰：

| 阶段 | 做什么 | 不做什么 | 退出条件 |
| --- | --- | --- | --- |
| 阶段1 | 只移动（`linear.x/y`） | 不转 | 位置误差 < 3cm |
| 阶段2 | 只旋转（`angular.z`） | 不移动 | 朝向误差 < 0.10rad（5.7°） |

**核心逻辑**：

1. 每 0.03 秒读取一次 `/odom` 当前位姿
2. 阶段1：计算位置误差 → 旋转到车体坐标 → 发 `linear.x/y`（不转）
3. 阶段2：计算朝向误差 → 发 `angular.z`（不动）
4. 超时 10 秒放弃

**为什么用闭环不用开环？** 开环（固定速度 + sleep）会因为摩擦、惯性、地面不平导致距离不准。闭环（实时读 odom 调整）能保证每次都精准到位。

### 9.3 在 `send_ground_nav_goal()` 中调用

```python
result = result_future.result()
if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
    return False
# Nav2 到达后做闭环精修，确保精准进入任务点
self.refine_ground_pose(x, y, yaw)
return True
```

### 9.4 三阶段控制的分工

| 阶段 | 控制者 | 做什么 | 速度 | 精度 | 耗时 |
| --- | --- | --- | --- | --- | --- |
| 1. 远距离导航 | Nav2 | 从起点开到目标点附近 | 0.75 m/s | ±3cm | 主要耗时 |
| 2. 位置精修 | 你的代码 | 只移动，对准 xy 坐标 | 0.08 m/s | ±3cm | ~1秒 |
| 3. 朝向精修 | 你的代码 | 只旋转，对准 yaw 朝向 | 0.25 rad/s | ±5.7° | ~1秒 |

Nav2 负责"快速开过去"，你的代码负责"最后停得准"。精修只在最后几厘米用低速，几乎不增加完成时间。

> **关键设计**：位置精修和朝向精修分开做，避免麦克纳姆轮旋转时的位置漂移导致"到点后反复转很久"的振荡问题。

---

## 10. 常见问题排查

| 现象 | 可能原因 | 解决 |
| --- | --- | --- |
| `action server 未就绪` | Nav2 还没启动完 | 多等几秒，或检查 `nav2_ground.launch.py` 终端有无报错 |
| 目标点无法到达 | 点在障碍物内/太靠墙 | 把目标点往空旷处挪一点 |
| 机器人原地转圈 | 定位漂移 / 雷达数据异常 | 检查 `/scan` 话题是否正常 |
| 路径规划失败 | 起点和终点之间被障碍完全隔开 | 检查地图或目标点坐标 |
| 到达判定太慢 | `xy_goal_tolerance` 太小 | 适当调大容差 |
| **小车横着/斜着走** | （正常现象）小车是麦克纳姆轮，全向运动 | 不是 bug，麦克纳姆轮可以横移 |
| **到点后朝向不对** | `yaw_goal_tolerance` 太大（默认 3.14 不检查朝向） | 改为 0.10，确保朝向也到位 |
| **进不去任务点方块** | yaw 设错，小车 44cm 边进不去 40cm 边 | 按"任务点朝向分析"章节设置正确 yaw |
| **到点位置偏差大** | `trans_stopped_velocity` 太大 + 缺少精修 | `trans_stopped_velocity` 改 0.02 + 加闭环精修（不要降速，影响完成时间） |
| **到点后旋转很久（振荡）** | 位置和朝向同时控制，旋转时位置漂移导致死循环 | 改用两阶段精修：先修位置再修朝向（见 9.2 节） |

---

## 11. 与飞行阶段的衔接

地面导航完成后，通常的衔接步骤：

1. 地面阶段：调用 `navigate_to_pose` 到达各巡检点
2. 切换到飞行：停止发布 `/cmd_vel`，改发 `/uav/cmd_vel`
3. 起飞：发布正 `linear.z`，或订阅 `/uav/odom` 做高度闭环
4. 飞行巡检：用 `/uav/cmd_vel` 控制到窗户/空中任务点
5. 降落：发布负 `linear.z` 直到接地

地面和飞行共享同一个机器人模型，二者通过不同话题控制（`/cmd_vel` vs `/uav/cmd_vel`），切换时注意**先停止另一个通道的速度**。
