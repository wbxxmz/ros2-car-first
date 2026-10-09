#!/usr/bin/env python3

import math
import time
from typing import  Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener


GroundWaypoint = Tuple[float, float, float]


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


class MinimalUserNode(Node):
    """A small template showing the simulator interfaces without solving the task."""

    def __init__(self) -> None:
        super().__init__("minimal_user_node")
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        '''self.declare_parameter("takeoff_seconds", 2.6)
        self.declare_parameter("takeoff_speed", 0.25)
        self.declare_parameter("move_seconds", 2.0)
        self.declare_parameter("move_speed", 0.12)
        self.declare_parameter("hover_seconds", 6.0)
        self.declare_parameter("land_seconds", 3.2)
        self.declare_parameter("land_speed", -0.20)
        self.declare_parameter("cmd_rate_hz", 25.0)'''

        self.ground_task_waypoints: Dict[str, GroundWaypoint] = {
            "ground_0": (1.125, -1.350, 0.0),
            "ground_1": (0.000, -1.350, 0.0),
            "ground_2": (0.000, -2.700, 0.0),
            "ground_3": (1.125, -1.925, math.pi/2),
            "ground_4": (1.125, -0.775, math.pi/2),
            "ground_5": (0.000, 0.000, 0.0),
        }
        self.ground_odom: Optional[Odometry] = None
        self.uav_odom: Optional[Odometry] = None

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action_name").value)
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.uav_cmd_vel_pub = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.laser_pub = self.create_publisher(Bool, "/uav/laser_fire", 10)
        # 裁判系统声明话题
        self.current_target_pub = self.create_publisher(String, "/referee/current_target", 10)
        self.cruise_event_pub = self.create_publisher(String, "/referee/cruise_event", 10)
        self.create_subscription(Odometry, "/odom", self.on_ground_odom, 10)
        self.create_subscription(Odometry, "/uav/odom", self.on_uav_odom, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_ground_odom(self, msg: Odometry) -> None:
        self.ground_odom = msg

    def on_uav_odom(self, msg: Odometry) -> None:
        self.uav_odom = msg
    def yaw_from_quaternion(self,qx,qy,qz,qw):
        siny_cosp = 2.0 * (qw *qz + qx *qy)
        cosy_cosp = 1.0 - 2.0 * (qy *qy + qz *qz)
        return math.atan2(siny_cosp, cosy_cosp)
    def get_ground_pose(self):
        if self.ground_odom is None:
            return None
        p = self.ground_odom.pose.pose.position
        o = self.ground_odom.pose.pose.orientation
        yaw = self.yaw_from_quaternion(o.x,o.y,o.z,o.w)
        return p.x,p.y,yaw

    def get_ground_pose_map(self):
        """通过 tf 获取 map 坐标系下的小车位姿，与 Nav2 目标点坐标系一致。"""
        try:
            tf = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
            t = tf.transform.translation
            r = tf.transform.rotation
            yaw = self.yaw_from_quaternion(r.x, r.y, r.z, r.w)
            return t.x, t.y, yaw
        except Exception:
            return None

    def refine_ground_pose(self,x,y,yaw,
                            pos_tol=0.05,
                            yaw_tol=0.15,
                            max_xy_speed=0.35,
                            min_xy_speed=0.1,
                            max_yaw_speed=1.2,
                            min_yaw_speed=0.3,
                            timeout_sec=12.0):
        """精修：位置+朝向同时控制（仿官方 demo 的组合指令）。
        关键：仿真插件是力矩控制（wheel_torque=2.0），低速指令克服不了摩擦，
        必须用官方 demo 的高增益（xy=0.9, yaw=1.8）且 linear+angular 同时发。"""
        refine_start = time.monotonic()
        self.get_logger().info(f"[精修阶段] === 进入精修 === 目标=({x:.3f},{y:.3f},yaw={yaw:.2f}) "
                              f"超时={timeout_sec}s")
        deadline = time.monotonic() + timeout_sec
        kp_xy = 0.9    # 官方 demo xy_gain
        kp_yaw = 1.8   # 官方 demo yaw_gain
        dist = 999.0
        err_yaw = 999.0
        # 先发停止指令
        self.cmd_vel_pub.publish(Twist())
        time.sleep(0.2)

        pose = self.get_ground_pose()
        if pose is None:
            self.get_logger().warn(f"[精修阶段] 无法获取 odom 坐标，跳过精修。目标=({x:.3f},{y:.3f})")
            return False
        cx, cy, cyaw = pose
        self.get_logger().info(f"[精修阶段] 精修开始：当前=({cx:.3f},{cy:.3f},yaw={cyaw:.2f}) "
                              f"初始误差 dist={math.hypot(x-cx, y-cy):.3f}m")

        loop_count = 0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.01)
            pose = self.get_ground_pose()
            if pose is None:
                time.sleep(0.05)
                continue
            cx, cy, cyaw = pose
            err_x = x - cx
            err_y = y - cy
            dist = math.hypot(err_x, err_y)
            err_yaw = yaw - cyaw
            while err_yaw > math.pi:
                err_yaw -= 2 * math.pi
            while err_yaw < -math.pi:
                err_yaw += 2 * math.pi

            loop_count += 1
            if loop_count % 30 == 0:
                self.get_logger().info(f"[精修阶段] 精修中：dist={dist:.3f}m yaw_err={err_yaw:.3f}")

            # 位置和朝向都到位 → 成功
            if dist < pos_tol and abs(err_yaw) < yaw_tol:
                self.cmd_vel_pub.publish(Twist())
                elapsed = time.monotonic() - refine_start
                self.get_logger().info(f"[精修阶段] 精修完成：最终=({cx:.3f},{cy:.3f},yaw={cyaw:.2f}) "
                                       f"误差={dist:.3f}m yaw误差={err_yaw:.3f} 耗时={elapsed:.1f}s")
                return True

            # --- 位置控制（世界系速度 + 最小速度地板）---
            vx_w = kp_xy * err_x
            vy_w = kp_xy * err_y
            if dist >= pos_tol:
                cur_speed = math.hypot(vx_w, vy_w)
                if cur_speed < min_xy_speed:
                    vx_w = min_xy_speed * err_x / dist
                    vy_w = min_xy_speed * err_y / dist
            else:
                vx_w = 0.0
                vy_w = 0.0

            # --- 朝向控制（最小速度地板克服摩擦）---
            if abs(err_yaw) >= yaw_tol:
                wz = kp_yaw * err_yaw
                if abs(wz) < min_yaw_speed:
                    wz = min_yaw_speed if err_yaw > 0 else -min_yaw_speed
                wz = min(max_yaw_speed, max(-max_yaw_speed, wz))
            else:
                wz = 0.0

            # --- 世界→机体坐标 ---
            c, s = math.cos(cyaw), math.sin(cyaw)
            body_x = c * vx_w + s * vy_w
            body_y = -s * vx_w + c * vy_w

            cmd = Twist()
            cmd.linear.x = min(max_xy_speed, max(-max_xy_speed, body_x))
            cmd.linear.y = min(max_xy_speed, max(-max_xy_speed, body_y))
            cmd.angular.z = wz
            self.cmd_vel_pub.publish(cmd)
            time.sleep(0.03)

        self.cmd_vel_pub.publish(Twist())
        elapsed = time.monotonic() - refine_start
        self.get_logger().warn(f"[精修阶段] 精修超时：当前=({cx:.3f},{cy:.3f},yaw={cyaw:.2f}) "
                               f"误差={dist:.3f}m yaw误差={err_yaw:.3f} 耗时={elapsed:.1f}s")
        return False

    def send_ground_nav_goal(self, waypoint: GroundWaypoint,
                              nav_timeout_sec: float = 180.0,
                              target_name: Optional[str] = None) -> bool:
        """发送 Nav2 导航目标。整体超时 nav_timeout_sec 秒，超时取消 goal。
        target_name 不为 None 时，精修成功后发布到点声明到 /referee/current_target。"""
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("[Nav2阶段] navigate_to_pose action server 未就绪。")
            return False

        x, y, yaw = waypoint
        self.get_logger().info(f"[Nav2阶段] 开始导航到 ({x:.3f}, {y:.3f}, yaw={yaw:.2f}) "
                              f"整体超时={nav_timeout_sec}s")
        nav_start = time.monotonic()

        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        # 阶段1：发送 goal，等接受（最多 10s）
        send_future = self.nav_client.send_goal_async(goal)
        if not self.wait_for_future(send_future, timeout_sec=10.0):
            self.get_logger().error(f"[Nav2阶段] 发送 goal 超时（10s）")
            return False
        goal_handle = send_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f"[Nav2阶段] goal 被拒绝")
            return False
        self.get_logger().info(f"[Nav2阶段] goal 已接受，等待导航完成")

        # 阶段2：等导航结果，剩余时间作为整体超时
        remaining = nav_start + nav_timeout_sec - time.monotonic()
        if remaining <= 0:
            self.get_logger().warn(f"[Nav2阶段] goal 接受后已超时，取消 goal")
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return False
        result_future = goal_handle.get_result_async()
        if not self.wait_for_future(result_future, timeout_sec=remaining):
            # 整体超时，取消当前 goal
            elapsed = time.monotonic() - nav_start
            self.get_logger().warn(f"[Nav2阶段] 导航超时 {elapsed:.1f}s (上限 {nav_timeout_sec}s)，取消 goal")
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return False
        result = result_future.result()
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"[Nav2阶段] 导航未成功，status="
                                  f"{result.status if result else 'None'}")
            return False

        nav_elapsed = time.monotonic() - nav_start
        self.get_logger().info(f"[Nav2阶段] 导航完成，耗时 {nav_elapsed:.1f}s，进入精修阶段")
        # Nav2 到达后做精修（先旋转到目标朝向，再调位置）
        refine_ok = self.refine_ground_pose(x, y, yaw)
        if refine_ok and target_name is not None:
            # 精修成功才发布到点声明（"进入点位后"才声明）
            self.declare_target(target_name)
        return refine_ok

    def set_laser(self, enabled: bool) -> None:
        try:
            self.laser_pub.publish(Bool(data=enabled))
        except Exception:
            pass

    def declare_target(self, name: str) -> None:
        """发布到点声明到裁判系统（如 ground_2）。"""
        msg = String(data=name)
        self.current_target_pub.publish(msg)
        self.get_logger().info(f"[裁判声明] 到点声明 → /referee/current_target: '{name}'")

    def declare_event(self, name: str) -> None:
        """发布事件声明到裁判系统（如 window_1, landing_marker）。"""
        msg = String(data=name)
        self.cruise_event_pub.publish(msg)
        self.get_logger().info(f"[裁判声明] 事件声明 → /referee/cruise_event: '{name}'")

    def get_uav_pose(self):
        if self.uav_odom is None:
            return None
        p = self.uav_odom.pose.pose.position
        o = self.uav_odom.pose.pose.orientation
        yaw = self.yaw_from_quaternion(o.x,o.y,o.z,o.w)
        return p.x,p.y,p.z,yaw
    
    def fly_to(self,x,y,z,yaw=0.0,
               pos_tol=0.15,yaw_tol=0.15,
               max_xy_speed=0.12,max_z_speed=0.25,
               max_yaw_speed=0.5,timeout_sec=30.0) -> bool:
            deadline = time.monotonic() + timeout_sec   
            kp_xy = 0.8
            kp_z = 0.6
            kp_yaw = 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self,timeout_sec=0.0)
                pose = self.get_uav_pose()
                if pose is None:
                    self.get_logger().error("未获取到无人机位姿。")
                    time.sleep(0.05)
                    continue
                cx,cy,cz,cyaw = pose
                err_x = x - cx
                err_y = y - cy
                err_z = z - cz
                err_yaw = yaw - cyaw
                while err_yaw > math.pi:
                    err_yaw -= 2 * math.pi
                while err_yaw < -math.pi:
                    err_yaw += 2 * math.pi
                if (abs(err_x) < pos_tol and abs(err_y) < pos_tol and abs(err_z) < pos_tol and abs(err_yaw) < yaw_tol):
                    self.get_logger().info("到达目标位置。")
                    self.uav_cmd_vel_pub.publish(Twist())
                    return True
                body_x = err_x * math.cos(-cyaw) - err_y * math.sin(-cyaw)
                body_y = err_x * math.sin(-cyaw) + err_y * math.cos(-cyaw)
                cmd = Twist()
                cmd.linear.x = min(max_xy_speed, max(-max_xy_speed, body_x * kp_xy))
                cmd.linear.y = min(max_xy_speed, max(-max_xy_speed, body_y * kp_xy))
                cmd.linear.z = min(max_z_speed, max(-max_z_speed, err_z * kp_z))
                cmd.angular.z = min(max_yaw_speed, max(-max_yaw_speed, err_yaw * kp_yaw))
                self.uav_cmd_vel_pub.publish(cmd)
                time.sleep(0.05)
            self.uav_cmd_vel_pub.publish(Twist())
            return False

    def stop_ground(self) -> None:
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def stop_uav(self) -> None:
        try:
            self.uav_cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        self.set_laser(False)

    def wait_for_future(self, future, timeout_sec: Optional[float] = None) -> bool:
        """等待 future 完成。timeout_sec 为整体超时（秒），None 表示无限等待。"""
        deadline = (time.monotonic() + timeout_sec) if timeout_sec is not None else None
        while rclpy.ok() and not future.done():
            if deadline is not None and time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return future.done()

    def takeoff(self, height=1.0):
        pose = self.get_uav_pose()
        if pose is None:
            self.get_logger().error("takeoff未获取到无人机位姿。")
            return False
        _,_,_,cyaw = pose
        self.get_logger().info(f"起飞到高度 {height:.2f}m")
        return self.fly_to(pose[0],pose[1],height,cyaw)
    def land(self) -> bool:
        pose = self.get_uav_pose()
        if pose is None:
            self.get_logger().error("land未获取到无人机位姿。")
            return False
        self.get_logger().info("降落中...")
        return self.fly_to(pose[0], pose[1], 0.05, pose[3])

    def hover(self, seconds: float) -> None:
        """悬停指定秒数，期间持续发零速度，避免 cmd_timeout 超时。"""
        deadline = time.monotonic() + seconds
        zero = Twist()
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.0)
            self.uav_cmd_vel_pub.publish(zero)
            time.sleep(0.05)   # 20Hz

    def run(self) -> None:
        for name, waypoint in self.ground_task_waypoints.items():
            self.get_logger().info(f"导航到 {name}：{waypoint}。")
            if not self.send_ground_nav_goal(waypoint, target_name=name):
                self.get_logger().error(f"导航目标 {name} 失败。")
                continue
        self.get_logger().info("===地面巡检任务完成===")
        self.get_logger().info("===开始飞行===")

        # 步骤1：起飞到 1m
        if not self.takeoff(height=1.0):
            self.get_logger().error("起飞失败，终止飞行阶段。")
            return
        self.hover(1.0)  # 稳定一下（不能用 sleep，会触发 cmd_timeout）

        # 步骤2：飞到窗户1识别位（窗框西面正前方 0.45m——识别图贴在框面上，不能贴框悬停）
        if not self.fly_to(1.400, -0.3275, 1.0, 0.0):
            self.get_logger().warn("未到达窗户1识别位，继续后续流程。")
        self.hover(0.5)

        # 步骤3：窗户1声明 → /referee/cruise_event: "window_1"
        self.declare_event("window_1")

        # 步骤4：窗户1识别图片发话术
        # TODO: 识别阶段实现——识别 window_target_1 的图片内容并喊话

        # 步骤5：两步穿越窗户1——先对准，再水平直线穿越
        # 5a 对准：飞到窗户1西侧 0.3m，降到穿越高度悬停，对准窗户中心
        if not self.fly_to(1.625, -0.3275, 0.7, 0.0, pos_tol=0.08):
            self.get_logger().warn("窗户1对准失败，继续后续流程。")
        self.hover(0.5)
        # 5b 穿越：保持高度水平直线穿过窗户，停在窗户东侧 0.4m（完全脱离窗框）
        if not self.fly_to(2.325, -0.3275, 0.7, 0.0, pos_tol=0.08):
            self.get_logger().warn("穿越窗户1失败，继续后续流程。")

        # 步骤6：飞到空中点1
        if not self.fly_to(3.080, -0.320, 1.0, 0.0):
            self.get_logger().warn("未到达空中点1，继续后续流程。")
        self.hover(0.5)

        # 步骤7：空中点1识别+灯光
        # TODO: 识别阶段实现——识别 + 开灯1-3秒 + 关灯

        # 步骤8：飞到空中点2
        if not self.fly_to(3.080, -1.355, 1.0, 0.0):
            self.get_logger().warn("未到达空中点2，继续后续流程。")
        self.hover(0.5)

        # 步骤9：空中点2识别+灯光
        # TODO: 识别阶段实现——识别 + 开灯1-3秒 + 关灯

        # 步骤10：飞到窗户2识别位（窗框东面正前方 0.45m）
        if not self.fly_to(2.450, -2.3725, 1.0, 0.0):
            self.get_logger().warn("未到达窗户2识别位，继续后续流程。")
        self.hover(0.5)

        # 步骤11：窗户2声明+识别+两步穿越
        self.declare_event("window_2")
        # TODO: 识别阶段实现——识别 window_target_2 的图片内容并喊话
        # 11a 对准：飞到窗户2东侧 0.3m，降到穿越高度悬停，对准窗户中心
        if not self.fly_to(2.225, -2.3725, 0.7, 0.0, pos_tol=0.08):
            self.get_logger().warn("窗户2对准失败，继续后续流程。")
        self.hover(0.5)
        # 11b 穿越：保持高度水平直线穿过窗户，停在窗户西侧 0.4m（完全脱离窗框）
        if not self.fly_to(1.525, -2.3725, 0.7, 0.0, pos_tol=0.08):
            self.get_logger().warn("穿越窗户2失败，继续后续流程。")

        # 步骤12：飞回起点上方（指挥中心 0,0），避免在窗户2位置原地降落撞框
        if not self.fly_to(0.0, 0.0, 1.0, 0.0):
            self.get_logger().warn("未返回起点上方，继续降落流程。")
        self.hover(0.5)

        # 步骤13：降落声明 → /referee/cruise_event: "landing_marker"
        self.declare_event("landing_marker")

        # 步骤14：降落（在起点上方垂直降）
        if not self.land():
            self.get_logger().error("降落失败。")
            return
        self.stop_uav()
        self.get_logger().info("=== 飞行阶段完成 ===")

def main(args=None) -> None:
    rclpy.init(args=args)
    node = MinimalUserNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.stop_ground()
            node.stop_uav()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
