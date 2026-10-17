#!/usr/bin/env python3

import math
import os
import random
import time
from typing import  Dict, Optional, Tuple

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

# 识别依赖：缺 OpenCV/numpy 只影响识别，导航与声明流程照常运行
# 注意：不用 cv_bridge——环境里的 cv_bridge 按 NumPy 1.x 编译，系统 NumPy 2.2.6 下
# imgmsg_to_cv2 必现段错误（_ARRAY_API not found，已实测复现），图像手动拆包即可
try:
    import cv2
    import numpy as np
    from ican_inspection_examples.recognizer import Recognizer
    HAS_VISION = True
except ImportError:
    HAS_VISION = False


GroundWaypoint = Tuple[float, float, float]

# === 比赛规则硬边界（依据 docs/地面导航禁区定义与违规操作判定.md v1.1）===
# 窗户竖条物理范围 x∈[1.845,2.005]（全 y 段规则禁入，唯一例外：飞行阶段
# 以窗户内径高度水平穿越 window_1/window_2）
GROUND_X_MAX = 1.70      # 地面阶段西半区硬边界（竖条西沿 1.845 − 机体半长 0.17，取 1.70）
GROUND_X_MIN = -0.55     # 西墙内沿 −0.775 + 半机 + 0.10m 裕量
GROUND_Y_MIN = -3.10     # 南墙内沿 −3.350 同理
GROUND_Y_MAX = 0.40      # 北墙内沿 0.650 同理
STRIP_X_MIN = 1.845      # 窗户竖条西沿
STRIP_X_MAX = 2.005      # 窗户竖条东沿
CROSS_Z_MIN = 0.45       # 合法窗户穿越高度带（内径 z∈[0.40,1.15]，两侧留 5cm）
CROSS_Z_MAX = 1.10
# 窗户开口 y 带（合法穿越通道仅此两条；两窗之间缝隙 y∈[-1.6725,-1.0275] 与
# 开口外区域一律禁入）。守卫判定在开口内沿各收缩 5cm，避免贴边抖动误报
W1_OPENING_Y = (-0.7775, 0.1225)    # 窗1开口 y∈[-0.8275,0.1725]
W2_OPENING_Y = (-2.8225, -1.9225)   # 窗2开口 y∈[-2.8725,-1.8725]
HOME_X = 0.0             # 指挥中心（官网任务二：地面任务后返回此处才准起飞）
HOME_Y = 0.0
TAKEOFF_HOME_RADIUS = 0.30   # 起飞位置距指挥中心的判定半径
GROUND_TAKEOFF_Z = 0.15      # 地面阶段异常起飞判据（地面正常 z≈0.055）
MONITOR_PERIOD_SEC = 0.1     # 运行监测/守卫周期 10Hz


def quaternion_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


# 地面/空中共用 5 类话术，窗户单独 2 类（关键词必须与判分细则一字不差）
GROUND_SPEECH = {
    "坍塌废墟": "发现坍塌废墟，地面巡航任务点确认，将巡航信息发送回指挥中心。",
    "滑坡塌方": "发现滑坡塌方，地面巡航任务点确认，将巡航信息发送回指挥中心。",
    "被困人员": "发现被困人员，地面巡航任务点确认，将巡航信息发送回指挥中心。",
    "受损车辆": "发现受损车辆，地面巡航任务点确认，将巡航信息发送回指挥中心。",
    "救援营地": "发现救援营地，地面巡航任务点确认，将巡航信息发送回指挥中心。",
}
AERIAL_SPEECH = "发现空中巡航任务点，识别目标为{cls}，执行灯光提示，将巡航信息发送回指挥中心。"
WINDOW_SPEECH = {
    "救援无人机": "发现窗户上方目标为救援无人机，执行下降避让，穿越窗户。",
    "飞鸟": "发现窗户上方目标为飞鸟，执行下降避让，穿越窗户。",
}


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
            "ground_1": (0.000, -1.350, math.pi),   # 与识别停点同朝向，免 180° 调头
            "ground_2": (0.000, -2.700, math.pi),   # 同上；板 0.55×0.40，朝西完全进入无影响
            "ground_3": (1.125, -1.925, math.pi/2),
            "ground_4": (1.125, -0.775, -math.pi/2),  # 与识别停点同朝向（北侧看朝 -y），免调头
        }
        # 地面识别停点：任务点前 0.85m、正对目标。相机水平朝前，
        # 开到点上反而看不到脚下的板，必须隔开一段距离才拍得全。
        # 停点方向按现场选：哪边空旷就从哪边看（避开墙、窗户和其他板）。
        self.ground_recog_stops: Dict[str, GroundWaypoint] = {
            "ground_0": (0.275, -1.350, 0.0),            # 西侧看，朝 +x
            "ground_1": (0.850, -1.350, math.pi),        # 东侧看，朝 -x（西侧是场地边界）
            "ground_2": (0.850, -2.700, math.pi),        # 东侧看，朝 -x
            "ground_3": (1.125, -2.775, math.pi/2),      # 南侧看，朝 +y
            "ground_4": (1.125, 0.075, -math.pi/2),      # 北侧看，朝 -y
        }
        # 地面板几何（中心 x/y + 板面尺寸），与 inspection_task1.world 一致
        self.ground_boards: Dict[str, Tuple[float, float, float, float]] = {
            "ground_0": (1.125, -1.350, 0.55, 0.40),
            "ground_1": (0.000, -1.350, 0.55, 0.40),
            "ground_2": (0.000, -2.700, 0.55, 0.40),
            "ground_3": (1.125, -1.925, 0.40, 0.55),
            "ground_4": (1.125, -0.775, 0.40, 0.55),
        }
        self.ground_odom: Optional[Odometry] = None
        self.uav_odom: Optional[Odometry] = None
        # 空中识别结果暂存：识别必须在西半区识别停点做（成像几何要求），但话术
        # 推迟到东半区任务点上空"真正到达"后再发——话术"发现空中巡航任务点"是在
        # 任务点上空才成立的动作，在识别停点发属未到先报（v1.7）
        self.aerial_classes: Dict[str, str] = {}
        # 空中识别停点：机载相机水平朝前（半视场 40°），装在机头前 0.39m、高 0.13m——
        # 悬在任务点 (3.080, y, 1.0) 正上方完全看不到脚下的板，必须停在板前靠俯角入画。
        # v1.5：识别悬停高度 0.40 → 0.80m（满足"飞行高度不得低于 80cm"）。
        # v1.6：停点从 x=1.00 东移到 x=1.30——x=1.00 时相机 (1.39,y,0.93) 距板面 1.69m，
        # 板在画面里面积只剩旧几何（z=0.40、距 1.14m）的约 1/2.2，LBP 细节不足，
        # 重训后票差仍只有 0.43/0.50（阈值 0.5，旧几何历史 0.72~3.71）。
        # x=1.30 时相机 (1.69,y,0.93) 到板近边 2.905 距 1.215m、俯角 37.5°→画面行 774；
        # 远边 3.255 距 1.565m、30.7°→行 707，整板落在画面下带 ROI[h-250:]（行 550~800）内，
        # 且离下沿还有 26 行余量（x=1.40 时近边就到行 798 会压边，故取 1.30）。
        # 视线过窗1开口高度 0.81(西沿)→0.69(东沿)，稳在窗洞 z∈[0.40,1.15] 内；
        # aerial_2 视线走两窗之间的空缝隙（该处无墙体，仅禁穿越）。机体 x=1.30 不进
        # 竖条 x∈[1.845,2.005]（最前端 1.475，留 0.37m）。
        # ⚠️ 停点在西半区属合法位置（识别按话术计分、不限悬停点）。两次空中识别仍全部
        # 前置到穿窗1 之前完成（见 run() 步骤5-6），无任何为识别而换半区的往返航路；
        # 两窗缝隙 y∈[-1.6725,-1.0275] 全程禁越（V6），视线可穿。
        # ⚠️ 几何变了 → 必须用 collect_dir 在新停点重采（采集抖动对称 ±0.12、每点 8 帧，
        # 覆盖运行时 ±0.08 的到位偏差）并重训 aerial_recognizer.npz。
        self.aerial_recog_stops: Dict[str, Tuple[float, float, float, float]] = {
            "aerial_1": (1.30, -0.320, 0.80, 0.0),
            "aerial_2": (1.30, -1.355, 0.80, 0.0),
        }

        self.nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter("nav_action_name").value)
        )
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.uav_cmd_vel_pub = self.create_publisher(Twist, "/uav/cmd_vel", 10)
        self.laser_pub = self.create_publisher(Bool, "/uav/laser_fire", 10)
        # 裁判系统声明话题
        self.current_target_pub = self.create_publisher(String, "/referee/current_target", 10)
        self.cruise_event_pub = self.create_publisher(String, "/referee/cruise_event", 10)
        # 识别话术话题
        self.detections_pub = self.create_publisher(String, "/inspection/detections", 10)
        self.create_subscription(Odometry, "/odom", self.on_ground_odom, 10)
        self.create_subscription(Odometry, "/uav/odom", self.on_uav_odom, 10)
        # 相机：回调只存最新帧，识别放任务流程里做（回调做重活会拖慢其他话题）
        self.ground_image_msg: Optional[Image] = None
        self.uav_image_msg: Optional[Image] = None
        # LBP+LDA 分类器（识别核心，见 recognizer.py 与《图片识别合并方案》）
        self.declare_parameter("recognizer_model", "")    # 地面 5 类模型，空=包内默认
        self.declare_parameter("aerial_recognizer_model", "")  # 空中专用模型（域差大，与地面混训会互相拖垮）
        self.declare_parameter("window_recognizer_model", "")  # 窗户 2 类模型
        self.declare_parameter("recognizer_margin", 0.5)  # 票差阈值，低于判未识别
        # 采集目录：设置后每次识别把矫正/裁剪后的 patch 存盘（配合 train_recognizer.py 采集训练数据）
        self.declare_parameter("collect_dir", "")
        self.declare_parameter("collect_tag", "")         # 文件名附加标记（如 seed07，便于按 seed 验证）
        self.ground_recognizer: Optional["Recognizer"] = None
        self.aerial_recognizer: Optional["Recognizer"] = None
        self.window_recognizer: Optional["Recognizer"] = None
        if HAS_VISION:
            self.create_subscription(Image, "/camera/image", self.on_ground_image, 10)
            self.create_subscription(Image, "/uav/camera/image", self.on_uav_image, 10)
            margin = float(self.get_parameter("recognizer_margin").value)
            pkg_dir = os.path.dirname(os.path.abspath(__file__))
            self._load_recognizer(
                "recognizer_model", os.path.join(pkg_dir, "models", "ground_recognizer.npz"),
                margin, "ground")
            self._load_recognizer(
                "aerial_recognizer_model", os.path.join(pkg_dir, "models", "aerial_recognizer.npz"),
                margin, "aerial")
            self._load_recognizer(
                "window_recognizer_model", os.path.join(pkg_dir, "models", "window_recognizer.npz"),
                margin, "window")
        else:
            self.get_logger().error(
                "[识别] 缺少 OpenCV/numpy，识别不可用；"
                "请安装：sudo apt install python3-opencv"
            )
        self.collect_dir = str(self.get_parameter("collect_dir").value)
        self.collect_tag = str(self.get_parameter("collect_tag").value)
        # 灯光控制（空中点：识别 → 话术 → 开灯1-3秒 → 关灯）
        self.light_pub = self.create_publisher(Bool, "/uav/light_on", 10)
        # 裁判系统抽签结果（从 5 个地面点随机抽 3 个，如 "ground_2,ground_4,ground_1"）
        self.selected_ground_points: Optional[list] = None
        self.create_subscription(String, "/referee/ground_tasks", self.on_ground_tasks, 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # === 规则守卫与运行监测（v1.1：地面禁飞 / 西半区硬边界 / 竖条穿越合法性）===
        self.flight_phase = False          # False=地面阶段；仅"回到指挥中心后"才置 True
        self.ground_violation: Optional[str] = None  # 地面阶段首个硬违规代码，置位后地面流程终止
        self._violation_last_log: Dict[str, float] = {}
        self._uav_air_since: Optional[float] = None  # 地面阶段 uav z 越限的持续起始时刻
        self._last_ground_cmd = Twist()
        self._last_uav_cmd = Twist()
        # 运行全程记录：t、阶段、双机位姿、双路速度指令、违规代码——
        # 复现"地面阶段真起飞"时可直接按时间轴定位诱因
        self._monitor_fp = None
        try:
            monitor_path = f"/tmp/run_monitor_{int(time.time())}.csv"
            self._monitor_fp = open(monitor_path, "w", buffering=1)
            self._monitor_fp.write(
                "t,phase,gx,gy,gz,ux,uy,uz,cmd_gx,cmd_gy,cmd_wz,"
                "cmd_ux,cmd_uy,cmd_uz,violation\n")
            self.get_logger().info(f"[监测] 运行全程记录 → {monitor_path}")
        except Exception as exc:
            self.get_logger().warn(f"[监测] 创建记录文件失败：{exc}")
        self.create_timer(MONITOR_PERIOD_SEC, self.monitor_callback)

    def on_ground_odom(self, msg: Odometry) -> None:
        self.ground_odom = msg

    def on_uav_odom(self, msg: Odometry) -> None:
        self.uav_odom = msg

    # ------------------------------------------------------------------
    # 指令发布封装：缓存最新指令供监测记录；飞控在地面阶段做非零拦截
    # ------------------------------------------------------------------
    def publish_ground_cmd(self, cmd: Twist) -> None:
        self._last_ground_cmd = cmd
        self.cmd_vel_pub.publish(cmd)

    def publish_uav_cmd(self, cmd: Twist) -> None:
        is_zero = all(abs(v) < 1e-9 for v in (
            cmd.linear.x, cmd.linear.y, cmd.linear.z,
            cmd.angular.x, cmd.angular.y, cmd.angular.z))
        # V1 守卫：地面阶段一切非零飞控指令都属违规，直接拦截并补零速抑制
        if not self.flight_phase and not is_zero:
            self._register_ground_violation(
                "V1", f"地面阶段拦截非零飞控指令 "
                      f"(vx={cmd.linear.x:.3f},vy={cmd.linear.y:.3f},vz={cmd.linear.z:.3f})")
            self._last_uav_cmd = Twist()
            self.uav_cmd_vel_pub.publish(Twist())
            return
        self._last_uav_cmd = cmd
        self.uav_cmd_vel_pub.publish(cmd)

    # ------------------------------------------------------------------
    # 违规检测与记录
    # ------------------------------------------------------------------
    def report_violation(self, code: str, msg: str, throttle_sec: float = 1.0) -> None:
        """打印违规告警并落监测记录；同类告警按 throttle_sec 去抖。"""
        now = time.monotonic()
        if now - self._violation_last_log.get(code, 0.0) >= throttle_sec:
            self._violation_last_log[code] = now
            self.get_logger().warn(f"[违规-{code}] {msg}")

    def _register_ground_violation(self, code: str, msg: str) -> str:
        """地面阶段硬违规：只记首个代码、立即双路停车。"""
        if self.ground_violation is None:
            self.ground_violation = code
            self.get_logger().error(f"[违规拦截-{code}] {msg} —— 地面流程立即终止")
        else:
            self.report_violation(code, msg)
        try:
            self.publish_ground_cmd(Twist())
        except Exception:
            pass
        try:
            self._last_uav_cmd = Twist()
            self.uav_cmd_vel_pub.publish(Twist())
        except Exception:
            pass
        return code

    def check_ground_boundary(self) -> Optional[str]:
        """地面阶段位置硬守卫（竖条/外墙）。安全返回 None，越界返回违规代码。"""
        if self.ground_odom is None:
            return None
        p = self.ground_odom.pose.pose.position
        if p.x > GROUND_X_MAX:
            return self._register_ground_violation(
                "V2", f"地面位置 x={p.x:.3f} 越过西半区硬边界 x≤{GROUND_X_MAX:.2f}"
                      f"（窗户竖条 x∈[{STRIP_X_MIN},{STRIP_X_MAX}] 含缝隙全程禁入）")
        if p.x < GROUND_X_MIN or p.y < GROUND_Y_MIN or p.y > GROUND_Y_MAX:
            return self._register_ground_violation(
                "V7", f"地面位置 ({p.x:.3f},{p.y:.3f}) 越过外墙安全边界 "
                      f"(x≥{GROUND_X_MIN}, y∈[{GROUND_Y_MIN},{GROUND_Y_MAX}])")
        return None

    def monitor_callback(self) -> None:
        """10Hz：V1 地面异常起飞检测、V5 穿越高度断言、全程 CSV 记录。"""
        try:
            now = time.monotonic()
            # V1：地面阶段 uav 持续离地 0.5s → 判定异常起飞，零速抑制并终止地面流程
            if not self.flight_phase and self.uav_odom is not None:
                z = self.uav_odom.pose.pose.position.z
                if z > GROUND_TAKEOFF_Z:
                    if self._uav_air_since is None:
                        self._uav_air_since = now
                    elif now - self._uav_air_since >= 0.5 and self.ground_violation is None:
                        self._register_ground_violation(
                            "V1", f"地面阶段检测到起飞：/uav/odom z={z:.3f} > {GROUND_TAKEOFF_Z}"
                                  " 持续 0.5s（地面巡航禁止起飞）")
                else:
                    self._uav_air_since = None
            # 位置边界（V2/V7）
            if not self.flight_phase:
                self.check_ground_boundary()
            # V5/V6：飞行阶段位于竖条内必须是合法窗户穿越——
            # ① y 在窗1/窗2开口带内（缝隙、开口外一律违规）② z 在窗户内径高度带
            if self.flight_phase and self.uav_odom is not None:
                up = self.uav_odom.pose.pose.position
                if STRIP_X_MIN <= up.x <= STRIP_X_MAX:
                    in_w1 = W1_OPENING_Y[0] <= up.y <= W1_OPENING_Y[1]
                    in_w2 = W2_OPENING_Y[0] <= up.y <= W2_OPENING_Y[1]
                    if not (in_w1 or in_w2):
                        self.report_violation(
                            "V6", f"位于竖条内但 y={up.y:.3f} 不在任一窗户开口带 "
                                  f"（两窗缝隙/开口外穿越，规则禁止）")
                    elif not (CROSS_Z_MIN <= up.z <= CROSS_Z_MAX):
                        self.report_violation(
                            "V5", f"穿越窗户开口 y={up.y:.3f} 时高度 z={up.z:.3f} "
                                  f"不在合法带 [{CROSS_Z_MIN},{CROSS_Z_MAX}]"
                                  "（禁止从窗上方飞越/竖条内升降）")
            self._write_monitor_row(now)
        except Exception as exc:
            # 监测回调绝不能影响主任务
            self.get_logger().warn(f"[监测] 回调异常（不影响任务）：{exc}")

    def _write_monitor_row(self, now: float) -> None:
        if self._monitor_fp is None:
            return
        g = self.ground_odom.pose.pose.position if self.ground_odom is not None else None
        u = self.uav_odom.pose.pose.position if self.uav_odom is not None else None
        cg, cu = self._last_ground_cmd, self._last_uav_cmd

        def f3(v):
            return f"{v:.3f}" if v is not None else ""

        self._monitor_fp.write(
            f"{now:.3f},{'FLIGHT' if self.flight_phase else 'GROUND'},"
            f"{f3(g.x if g else None)},{f3(g.y if g else None)},{f3(g.z if g else None)},"
            f"{f3(u.x if u else None)},{f3(u.y if u else None)},{f3(u.z if u else None)},"
            f"{cg.linear.x:.3f},{cg.linear.y:.3f},{cg.angular.z:.3f},"
            f"{cu.linear.x:.3f},{cu.linear.y:.3f},{cu.linear.z:.3f},"
            f"{self.ground_violation or ''}\n")

    def close_monitor(self) -> None:
        if self._monitor_fp is not None:
            try:
                self._monitor_fp.close()
            except Exception:
                pass
            self._monitor_fp = None

    def ground_target_allowed(self, x: float, y: float) -> bool:
        """地面导航目标硬边界断言（发送 Nav2 goal 前调用）。"""
        return (x <= GROUND_X_MAX and x >= GROUND_X_MIN
                and GROUND_Y_MIN <= y <= GROUND_Y_MAX)

    def at_home(self, radius: float = TAKEOFF_HOME_RADIUS) -> bool:
        """是否在指挥中心判定半径内（优先 map 系位姿）。"""
        pose = self.get_ground_pose_map() or self.get_ground_pose()
        if pose is None:
            return False
        return math.hypot(pose[0] - HOME_X, pose[1] - HOME_Y) <= radius

    def on_ground_image(self, msg: Image) -> None:
        self.ground_image_msg = msg

    def on_uav_image(self, msg: Image) -> None:
        self.uav_image_msg = msg

    def _load_recognizer(self, param_name: str, default_path: str,
                         margin: float, tag: str) -> None:
        """加载分类器模型：参数路径优先，其次包内默认；没有模型不阻塞导航流程。"""
        path = str(self.get_parameter(param_name).value) or default_path
        if os.path.exists(path):
            try:
                setattr(self, f"{tag}_recognizer", Recognizer(path, margin))
                label = {"ground": "地面", "aerial": "空中", "window": "窗户"}.get(tag, tag)
                self.get_logger().info(
                    f"[识别] 已加载{label}模型：{path}"
                    f"（票差阈值 {margin}）")
            except Exception as exc:
                self.get_logger().error(f"[识别] 模型加载失败 {path}：{exc}")
        else:
            self.get_logger().warn(
                f"[识别] 未找到模型 {path}，{tag} 识别将走兜底；"
                f"请先用 collect_dir 采集数据并运行 train_recognizer.py。")

    def _maybe_collect(self, point_name: str, patch) -> None:
        """采集钩子：collect_dir 参数非空时，把识别输入的 patch 存盘当训练数据。

        文件名 {点位}_{标记}_{时间戳}.png，类别由 train_recognizer.py
        查 texture_assignment.json 反推（采的时候不用知道答案）。
        """
        if not self.collect_dir or not HAS_VISION:
            return
        try:
            os.makedirs(self.collect_dir, exist_ok=True)
            # 毫秒级时间戳：多帧采集时秒级会同名覆盖
            stamp = f"{int(time.time() * 1000)}"
            tag = f"_{self.collect_tag}" if self.collect_tag else ""
            cv2.imwrite(
                os.path.join(self.collect_dir, f"{point_name}{tag}_{stamp}.png"), patch)
            self.get_logger().info(f"[采集] 已存 {point_name}{tag}_{stamp}.png")
        except Exception as exc:
            self.get_logger().warn(f"[采集] 存图失败：{exc}")

    def set_light(self, on: bool) -> None:
        self.light_pub.publish(Bool(data=on))

    def on_ground_tasks(self, msg: String) -> None:
        """裁判系统发布随机抽签的 3 个地面点位，如 "ground_2,ground_4,ground_1"。"""
        points = [name.strip() for name in msg.data.split(",") if name.strip()]
        valid = [name for name in points if name in self.ground_task_waypoints]
        if valid:
            self.selected_ground_points = valid
            self.get_logger().info(f"[裁判抽签] 地面点位顺序：{valid}")

    def wait_for_ground_tasks(self, timeout_sec: float = 10.0) -> list:
        """等待裁判抽签结果；当前仿真无裁判系统，超时后回退为全部 5 点默认顺序。"""
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and self.selected_ground_points is None:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.selected_ground_points is not None:
            return self.selected_ground_points
        self.get_logger().warn(
            f"[裁判抽签] {timeout_sec:.0f}s 内未收到 /referee/ground_tasks，"
            f"回退默认顺序：全部 5 个地面点。"
        )
        return list(self.ground_task_waypoints.keys())

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
                            pos_tol=0.07,
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
        # 硬守卫：已触发地面违规或目标越界，不再执行任何地面运动
        if self.ground_violation is not None:
            self.get_logger().error(f"[精修阶段] 已存在违规 {self.ground_violation}，拒绝精修。")
            return False
        if not self.ground_target_allowed(x, y):
            self.get_logger().error(
                f"[违规拦截] 精修目标 ({x:.3f},{y:.3f}) 越出西半区硬边界，拒绝执行。")
            self._register_ground_violation("V2", f"精修目标越界 x={x:.3f}")
            return False
        deadline = time.monotonic() + timeout_sec
        kp_xy = 0.9    # 官方 demo xy_gain
        kp_yaw = 1.8   # 官方 demo yaw_gain
        dist = 999.0
        err_yaw = 999.0
        # 先发停止指令
        self.publish_ground_cmd(Twist())
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
            # 实时硬守卫：越界（含被 Nav2/外力带入竖条）立即停车中止
            if self.check_ground_boundary() is not None:
                self.publish_ground_cmd(Twist())
                self.get_logger().error(
                    f"[精修阶段] 触发地面边界违规 {self.ground_violation}，精修中止。")
                return False
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
                self.publish_ground_cmd(Twist())
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
            self.publish_ground_cmd(cmd)
            time.sleep(0.03)

        self.publish_ground_cmd(Twist())
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
        # 硬守卫：地面导航目标必须在西半区边界内（防止东半区/竖条目标触发穿缝规划）
        if not self.ground_target_allowed(x, y):
            self.get_logger().error(
                f"[违规拦截] 地面导航目标 ({x:.3f},{y:.3f}) 超出西半区硬边界 "
                f"x∈[{GROUND_X_MIN},{GROUND_X_MAX}]，拒绝发送 Nav2 goal。")
            self._register_ground_violation("V2", f"Nav2 目标越界 ({x:.3f},{y:.3f})")
            return False
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
        # 自轮询（替代 wait_for_future）：导航途中持续跑边界/禁飞守卫，违规立即取消 goal
        nav_deadline = time.monotonic() + remaining
        while rclpy.ok() and not result_future.done() and time.monotonic() < nav_deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.ground_violation is not None:
                self.get_logger().error(
                    f"[违规拦截] Nav2 导航途中触发 {self.ground_violation}，取消当前 goal")
                try:
                    goal_handle.cancel_goal_async()
                except Exception:
                    pass
                self.publish_ground_cmd(Twist())
                return False
        if not result_future.done():
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

    def publish_detection(self, text: str) -> None:
        """发布识别话术到裁判系统 /inspection/detections。"""
        msg = String()
        msg.data = text
        self.detections_pub.publish(msg)
        self.get_logger().info(f"[识别] 已发布话术：{text}")

    def report_ground(self, detected_class: str) -> None:
        """地面点：按识别类别发话术。"""
        self.publish_detection(GROUND_SPEECH[detected_class])

    def image_to_gray(self, msg: Image):
        """sensor_msgs/Image → 灰度图（手动拆包，替代段错误的 cv_bridge）。"""
        if msg.encoding not in ("rgb8", "bgr8", "mono8"):
            self.get_logger().warn(f"[地面识别] 不支持的图像编码：{msg.encoding}")
            return None
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "mono8":
            return buf.reshape(msg.height, msg.width).copy()
        img = buf.reshape(msg.height, msg.width, 3).copy()
        code = cv2.COLOR_RGB2GRAY if msg.encoding == "rgb8" else cv2.COLOR_BGR2GRAY
        return cv2.cvtColor(img, code)

    def project_board_quad(self, rx: float, ry: float, ryaw: float,
                           bx: float, by: float, sx: float, sy: float):
        """把地面板四个角投影成像素坐标（相机：机体前 0.16m、高 0.11m、水平朝前）。

        用实测位姿反算板在画面里的位置，停点哪怕偏差十几厘米也不影响矫正结果。
        返回矩形四角顺序：左上(板西北角)、右上、右下、左下——与矫正目标矩形一一对应。
        """
        if not HAS_VISION:
            return None
        cam_fwd = 0.16                                  # 相机相对机体前向偏移
        cam_h = 0.055 + 0.11 - 0.012                    # 机体离地 + 相机安装高 - 板厚
        fx = fy = 400.0 / math.tan(1.3962634 / 2.0)     # 800px 画幅、水平视场 80°
        cx = cy = 400.0
        c, s = math.cos(ryaw), math.sin(ryaw)
        quad = []
        for wx, wy in ((bx - sx / 2, by - sy / 2), (bx + sx / 2, by - sy / 2),
                       (bx + sx / 2, by + sy / 2), (bx - sx / 2, by + sy / 2)):
            dx, dy = wx - rx, wy - ry
            fwd = c * dx + s * dy - cam_fwd      # 沿视线方向的深度
            left = -s * dx + c * dy              # 机体左向 → 图像左（u 变小）
            if fwd <= 0.05:                      # 板在相机背后或紧贴镜头
                return None
            quad.append((cx - fx * left / fwd, cy + fy * cam_h / fwd))
        return np.array(quad, dtype=np.float32)

    def recognize_ground(self, name: str) -> Optional[str]:
        """地面点识别：把画面里的地面板矫正成标准矩形后交给 LBP+LDA 分类器。

        板是平铺在地上的，相机水平朝前只能斜着看，画面里是透视梯形——
        先按几何把梯形矫正回矩形，再做特征分类（分类器学的是矫正后的纹理）。
        """
        if not HAS_VISION or self.ground_image_msg is None:
            self.get_logger().warn("[地面识别] 无相机数据，跳过识别。")
            return None
        pose = self.get_ground_pose()
        board = self.ground_boards.get(name)
        if pose is None or board is None:
            self.get_logger().warn(f"[地面识别] 缺少位姿或板面几何，跳过 {name}。")
            return None
        quad = self.project_board_quad(*pose, *board)
        if quad is None:
            self.get_logger().warn(f"[地面识别] {name} 板面不在相机视野内（投影失败）。")
            return None

        gray = self.image_to_gray(self.ground_image_msg)
        if gray is None:
            return None

        # 矫正尺寸按板面真实长宽比，保证纹理比例还原正确
        rect_w = 480
        rect_h = int(round(rect_w * board[3] / board[2]))
        src = np.array([[0, 0], [rect_w, 0], [rect_w, rect_h], [0, rect_h]], dtype=np.float32)
        homo = cv2.getPerspectiveTransform(quad, src)
        rect = cv2.warpPerspective(gray, homo, (rect_w, rect_h))
        # 调试留档：识别不理想时肉眼核对矫正结果
        cv2.imwrite(f"/tmp/ground_recog_{name}.png", rect)
        # 采集钩子：存矫正图当训练数据（训练侧再做旋转增广，保持与推理一致）
        self._maybe_collect(name, rect)

        if self.ground_recognizer is None:
            self.get_logger().warn("[地面识别] 无分类器模型，跳过识别。")
            return None
        # 材质里有 rotate 90，贴图方向未知——4 个旋转方向都试，取最高票差
        margin_x, margin_y = int(rect_w * 0.06), int(rect_h * 0.06)
        best_cls, best_margin = None, -1.0
        for k in range(4):
            cand = np.ascontiguousarray(np.rot90(rect, k))
            roi = cand[margin_y:rect_h - margin_y, margin_x:rect_w - margin_x]
            cls, margin = self.ground_recognizer.classify(roi)
            if margin > best_margin:
                best_cls, best_margin = cls, margin

        if best_cls is None:
            self.get_logger().warn(
                f"[地面识别] {name} 票差不足（最高 {best_margin:.2f}，"
                f"阈值 {self.ground_recognizer.margin_thresh}）。")
            return None
        self.get_logger().info(f"[地面识别] {name} → {best_cls}（票差 {best_margin:.2f}）")
        return best_cls

    def recognize_aerial(self, point_name: str) -> Optional[str]:
        """空中点识别：识别停点画面下方横带 ROI → 分类器（5 类同地面，共用模型）。

        停点几何下板面投影在画面下沿（见 aerial_recog_stops 注释），中心 ROI 会拍空。
        """
        if not HAS_VISION or self.uav_image_msg is None:
            self.get_logger().warn("[空中识别] 无相机数据，跳过识别。")
            return None
        gray = self.image_to_gray(self.uav_image_msg)
        if gray is None:
            return None
        h, w = gray.shape
        roi = gray[h - 250:, w // 2 - 180:w // 2 + 180]
        cv2.imwrite(f"/tmp/aerial_recog_{point_name}.png", roi)
        # 采集钩子放在模型检查前：采集运行时本来就没有模型
        self._maybe_collect(point_name, roi)
        # 空中优先用专用模型：地面矫正图与空中ROI带跨域距离远大于类间距离，
        # 混训时LDA被域差主导（实测混训空中票差0.03~0.44，专用模型1.1~1.65）
        rec = self.aerial_recognizer or self.ground_recognizer
        if rec is None:
            self.get_logger().warn("[空中识别] 无分类器模型，跳过识别。")
            return None
        # 与训练侧 4 旋转增广一致，逐旋转取最高票差（单帧直投实测会把
        # aerial_2 误判为坍塌废墟，4 旋转取优后 1.1+ 票差正确）
        best_cls, best_margin = None, -1.0
        for k in range(4):
            cand = np.ascontiguousarray(np.rot90(roi, k))
            ch, cw = cand.shape
            mx, my = int(cw * 0.06), int(ch * 0.06)
            r = cand[my:ch - my, mx:cw - mx]
            cls, margin = rec.classify(r)
            if margin > best_margin:
                best_cls, best_margin = cls, margin
        if best_cls is None:
            self.get_logger().warn(
                f"[空中识别] {point_name} 票差不足（最高 {best_margin:.2f}，"
                f"阈值 {rec.margin_thresh}）。")
            return None
        self.get_logger().info(f"[空中识别] {point_name} → {best_cls}（票差 {best_margin:.2f}）")
        return best_cls

    def recognize_window(self, window_name: str) -> Optional[str]:
        """窗户识别：识别图贴在窗框上部，取画面上半部中心 ROI → 窗户 2 类模型。"""
        if not HAS_VISION or self.uav_image_msg is None:
            self.get_logger().warn("[窗户识别] 无相机数据，跳过识别。")
            return None
        gray = self.image_to_gray(self.uav_image_msg)
        if gray is None:
            return None
        w = gray.shape[1]
        # 识别图贴在窗框上沿（中心 z=1.40，比相机高 0.27m）：投影在画面上半部，
        # ROI 取上方宽横带，覆盖识别停点 ±0.10m 距离浮动时的贴图投影范围
        roi = gray[0:300, w // 2 - 230:w // 2 + 230]
        cv2.imwrite(f"/tmp/window_recog_{window_name}.png", roi)
        self._maybe_collect(window_name, roi)
        if self.window_recognizer is None:
            self.get_logger().warn("[窗户识别] 无分类器模型，跳过识别。")
            return None
        cls, margin = self.window_recognizer.classify(roi)
        if cls is None:
            self.get_logger().warn(
                f"[窗户识别] {window_name} 票差不足（{margin:.2f}，"
                f"阈值 {self.window_recognizer.margin_thresh}）。")
            return None
        self.get_logger().info(f"[窗户识别] {window_name} → {cls}（票差 {margin:.2f}）")
        return cls

    def collect_multi_aerial(self, point_name: str, n: int = 4, interval: float = 0.4) -> None:
        """采集模式专用：空中停点悬停期间多抓 n 帧（悬停微漂移=天然增广）。
        空中每类训练图只有 2 张，票差 0.1~0.4 上不去；单帧采集不够。"""
        if not self.collect_dir or not HAS_VISION or self.uav_image_msg is None:
            return
        for i in range(n):
            gray = self.image_to_gray(self.uav_image_msg)
            if gray is None:
                return
            h, w = gray.shape
            roi = gray[h - 250:, w // 2 - 180:w // 2 + 180]
            self._maybe_collect(point_name, roi)
            if i < n - 1:
                self.hover(interval)

    def aerial_recognize_and_store(self, point_name: str) -> None:
        """空中识别停点（西半区）：只识别并暂存类别，不发话术。

        v1.7：话术推迟到步骤 9/10 到达任务点上空后再发（见 aerial_speak）。
        """
        cls = self.recognize_aerial(point_name)
        if cls is None:
            cls = "坍塌废墟"
            self.get_logger().warn(f"[空中识别] {point_name} 识别失败，按兜底类别上报。")
        self.aerial_classes[point_name] = cls

    def aerial_speak(self, point_name: str) -> None:
        """到达空中任务点上空后发话术（判分顺序：识别→话术→开灯）。"""
        cls = self.aerial_classes.get(point_name, "坍塌废墟")
        self.publish_detection(AERIAL_SPEECH.format(cls=cls))

    def aerial_light_signal(self) -> None:
        """空中任务点上空：开灯 2 秒 → 关灯（判分动作）。"""
        self.set_light(True)
        self.hover(2.0)
        self.set_light(False)

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
                    self.publish_uav_cmd(Twist())
                    return True
                body_x = err_x * math.cos(-cyaw) - err_y * math.sin(-cyaw)
                body_y = err_x * math.sin(-cyaw) + err_y * math.cos(-cyaw)
                cmd = Twist()
                cmd.linear.x = min(max_xy_speed, max(-max_xy_speed, body_x * kp_xy))
                cmd.linear.y = min(max_xy_speed, max(-max_xy_speed, body_y * kp_xy))
                cmd.linear.z = min(max_z_speed, max(-max_z_speed, err_z * kp_z))
                cmd.angular.z = min(max_yaw_speed, max(-max_yaw_speed, err_yaw * kp_yaw))
                self.publish_uav_cmd(cmd)
                time.sleep(0.05)
            self.publish_uav_cmd(Twist())
            return False

    def stop_ground(self) -> None:
        try:
            self.publish_ground_cmd(Twist())
        except Exception:
            pass

    def stop_uav(self) -> None:
        try:
            self.publish_uav_cmd(Twist())
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
            self.publish_uav_cmd(zero)
            time.sleep(0.05)   # 20Hz

    def run(self) -> None:
        # 按裁判抽签顺序导航地面点（无裁判系统时回退为全部 5 点）
        for name in self.wait_for_ground_tasks():
            waypoint = self.ground_task_waypoints[name]
            # 1) 先到识别停点（任务点前 0.85m 正对目标），板面才在相机视野里
            stop = self.ground_recog_stops[name]
            if self.collect_dir:
                # 采集模式加 ±0.15m 停点抖动，让训练数据覆盖比赛时的停点距离浮动
                stop = (stop[0] + random.uniform(-0.15, 0.15),
                        stop[1] + random.uniform(-0.15, 0.15), stop[2])
            self.get_logger().info(f"前往 {name} 识别停点 {stop}。")
            if not self.send_ground_nav_goal(stop, nav_timeout_sec=60.0):
                # 硬违规（越界/异常起飞）没有继续比赛的意义；普通导航失败保持原容错
                if self.ground_violation is not None:
                    self.get_logger().error(
                        f"前往 {name} 识别停点途中触发违规 {self.ground_violation}，任务终止。")
                    return
            if self.ground_violation is not None:
                self.get_logger().error(f"识别停点阶段已触发违规 {self.ground_violation}，任务终止。")
                return
            # 2) 识别 → 发话术（判分顺序：识别先于进点，话术先于到点声明）
            detected = self.recognize_ground(name)
            if detected is None:
                detected = "坍塌废墟"
                self.get_logger().warn(f"[地面识别] {name} 识别失败，按兜底类别上报。")
            self.report_ground(detected)
            # 3) 停点→任务点仅 0.85m 空地，不值得再启 Nav2 全套流程
            # （goal+全局规划+控制器启动每点白花 8~15s），直接精修直线挪过去（4~6s）。
            # ground_1/2 任务点朝向已与停点一致（π），无调头。
            self.get_logger().info(f"直线挪到 {name}：{waypoint}。")
            if self.refine_ground_pose(*waypoint):
                self.declare_target(name)
            else:
                self.get_logger().error(f"挪到 {name} 失败。")
                if self.ground_violation is not None:
                    self.get_logger().error(
                        f"进入任务点途中触发违规 {self.ground_violation}，任务终止。")
                    return
                continue
        self.get_logger().info("===地面巡检任务完成===")

        # 官网任务二：完成地面巡航后必须"返回指挥中心，从指挥中心开始起飞"。
        # 原实现是在最后一个抽签点原地起飞（流程违规），此处先地面导航回 (0,0)。
        self.get_logger().info("===返回指挥中心（官网任务二）===")
        home_waypoint: GroundWaypoint = (HOME_X, HOME_Y, 0.0)
        if not self.send_ground_nav_goal(home_waypoint, nav_timeout_sec=60.0):
            if self.ground_violation is not None:
                self.get_logger().error(
                    f"返回指挥中心途中触发违规 {self.ground_violation}，任务终止。")
            else:
                self.get_logger().error(
                    "返回指挥中心失败：规则禁止在非指挥中心位置起飞，任务终止。")
            return
        if not self.at_home():
            self.get_logger().error(
                f"当前位置距指挥中心超过判定半径 {TAKEOFF_HOME_RADIUS:.2f}m，"
                "禁止起飞（官网任务二），任务终止。")
            return
        # 至此才允许飞控指令与飞行流程
        self.flight_phase = True
        self.get_logger().info(f"===已在指挥中心（半径 {TAKEOFF_HOME_RADIUS:.2f}m 内），开始飞行===")

        # 步骤1：起飞到 1m
        if not self.takeoff(height=1.0):
            self.get_logger().error("起飞失败，终止飞行阶段。")
            return
        self.hover(1.0)  # 稳定一下（不能用 sleep，会触发 cmd_timeout）

        # ===== v1.4 识别路线重规划：识别全部前置在西半区，穿窗各一次 =====
        # 两个空中识别停点本就在西半区 x=1.55（透过窗1开口/两窗缝隙可见任务点贴图，
        # 识别几何与训练一致），只有"灯光提示"必须飞到东半区任务点正上方。
        # 因此：窗1识别 → 空1识别 → 空2识别 → 穿窗1(仅1次) → 空1/空2灯光 →
        # 窗2识别 → 穿窗2(仅1次) → 返回降落。穿窗 6→2 次，航程 16.9→11.4m，
        # 无任何为识别而换半区的航路，两窗缝隙全程无穿越机会（V6 不可能触发）。
        # 速度分层：巡航段 0.35m/s、停点接近 0.25m/s（均≤裁判限速 0.5），
        # 穿越与对准保持已验证的 0.12m/s 不变。
        # 步骤2：飞到窗户1识别位。贴图中心 (1.815, -0.3275, 1.40)、尺寸 0.36×0.22，
        # 相机在机头前 0.39m、比贴图中心低 0.27m——停 x=0.925 时距贴图 0.50m 全图入视场，
        # 且距窗框 0.89m（≥0.4m 安全距离）
        wx = 0.925 + (random.uniform(-0.10, 0.10) if self.collect_dir else 0.0)
        if not self.fly_to(wx, -0.3275, 1.0, 0.0, max_xy_speed=0.35):
            self.get_logger().warn("未到达窗户1识别位，继续后续流程。")
        self.hover(0.5)

        # 步骤3：窗户1声明 → /referee/cruise_event: "window_1"
        self.declare_event("window_1")

        # 步骤4：窗户1识别图片发话术（判分顺序：声明→识别→话术→下降穿越）
        win_cls = self.recognize_window("window_1")
        if win_cls is None:
            win_cls = "救援无人机"
            self.get_logger().warn("[窗户识别] window_1 识别失败，按兜底类别上报。")
        self.publish_detection(WINDOW_SPEECH[win_cls])

        # 步骤5：空中点1识别停点（识别前置）。停点 x=1.30、z=0.80（成像几何见
        # aerial_recog_stops 注释）。x=1.30 不进窗台投影（x∈[1.845,2.005]），机体最前端
        # 1.475 留 0.37m；悬停高度 0.80 满足"飞行高度不得低于 80cm"——识别悬停不属
        # "下降穿越"动作，故可用 0.80。窗1的"明显下降"由本步 1.0→0.80 与步骤7
        # 0.80→0.70 连续完成，此后机体保持 0.70 穿越，不回到识别高度
        sx = 1.30 + (random.uniform(-0.12, 0.12) if self.collect_dir else 0.0)
        if not self.fly_to(sx, -0.320, 0.80, 0.0, pos_tol=0.08, max_xy_speed=0.25):
            self.get_logger().warn("未到达空中点1识别停点，继续后续流程。")
        self.hover(0.5)
        if self.collect_dir:
            self.collect_multi_aerial("aerial_1", n=8)
        else:
            # 只识别并暂存：话术等到步骤9到达任务点上空再发（v1.7）
            self.aerial_recognize_and_store("aerial_1")

        # 步骤6：南下到空中点2识别停点（同高度同 x 平移，识别几何与训练一致）
        if not self.fly_to(sx, -1.355, 0.80, 0.0, pos_tol=0.08, max_xy_speed=0.25):
            self.get_logger().warn("未到达空中点2识别停点，继续后续流程。")
        self.hover(0.5)
        if self.collect_dir:
            self.collect_multi_aerial("aerial_2", n=8)
        else:
            # 只识别并暂存：话术等到步骤10到达任务点上空再发（v1.7）
            self.aerial_recognize_and_store("aerial_2")

        # 步骤7：窗1对准——先在识别停点 x=1.30（远离窗台、正下方无遮挡）垂降到
        # 穿越高度 0.70，再北上对准开口中心；升降只在 x=1.30 处垂直做，不贴近窗台
        if not self.fly_to(sx, -1.355, 0.70, 0.0, pos_tol=0.08):
            self.get_logger().warn("降至窗1穿越高度失败，继续后续流程。")
        if not self.fly_to(1.625, -0.3275, 0.70, 0.0, pos_tol=0.08, max_xy_speed=0.25):
            self.get_logger().warn("窗户1对准失败，继续后续流程。")
        self.hover(0.5)

        # 步骤8：穿越窗户1（本航线唯一一次东行穿越，高度 0.70∈[0.45,1.10] 合法带，
        # 开口 y=-0.3275）——识别已前置，判分顺序"声明→识别→话术→下降穿越"仍成立
        if not self.fly_to(2.325, -0.3275, 0.70, 0.0, pos_tol=0.08):
            self.get_logger().warn("穿越窗户1失败，继续后续流程。")

        # 步骤9：空中点1任务点上空 z=1.0（≥0.80m）：到达后才发话术 → 开灯
        # （判分顺序：识别(步骤5)→话术→开灯1-3秒→关灯；话术必须到任务点再发，
        # 识别停点在西半区只暂存类别，不提前播报）
        if not self.fly_to(3.080, -0.320, 1.0, 0.0, max_xy_speed=0.35):
            self.get_logger().warn("未到达空中点1，继续后续流程。")
        self.hover(0.5)
        if not self.collect_dir:
            self.aerial_speak("aerial_1")
        self.aerial_light_signal()

        # 步骤10：空中点2任务点——东半区内沿 x=3.08 直线南下（距竖条东沿 1.07m），
        # 到达后才发话术 → 开灯（同步骤9）
        if not self.fly_to(3.080, -1.355, 1.0, 0.0, max_xy_speed=0.35):
            self.get_logger().warn("未到达空中点2，继续后续流程。")
        self.hover(0.5)
        if not self.collect_dir:
            self.aerial_speak("aerial_2")
        self.aerial_light_signal()

        # 步骤11：飞到窗户2识别位。贴图中心 (2.035, -2.3725, 1.40) 朝东，从东侧看、
        # 机头必须朝西（yaw=π——之前机头朝 +x 是背对贴图）；停 x=2.925 时距贴图 0.50m
        wx = 2.925 + (random.uniform(-0.10, 0.10) if self.collect_dir else 0.0)
        if not self.fly_to(wx, -2.3725, 1.0, math.pi, max_xy_speed=0.35):
            self.get_logger().warn("未到达窗户2识别位，继续后续流程。")
        self.hover(0.5)

        # 步骤12：窗户2声明+识别+发话术
        self.declare_event("window_2")
        win_cls = self.recognize_window("window_2")
        if win_cls is None:
            win_cls = "救援无人机"
            self.get_logger().warn("[窗户识别] window_2 识别失败，按兜底类别上报。")
        self.publish_detection(WINDOW_SPEECH[win_cls])

        # 步骤13：两步穿越窗户2（本航线唯一一次西行穿越，高度 0.70、开口 y=-2.3725）
        # 13a 对准：飞到窗户2东侧，降到穿越高度悬停，对准窗户中心（机头保持朝西不转向）
        if not self.fly_to(2.225, -2.3725, 0.7, math.pi):
            self.get_logger().warn("窗户2对准失败，继续后续流程。")
        self.hover(0.5)
        # 13b 穿越：保持高度水平直线穿过窗户，停在窗户西侧 0.4m（完全脱离窗框）
        if not self.fly_to(1.525, -2.3725, 0.7, math.pi):
            self.get_logger().warn("穿越窗户2失败，继续后续流程。")

        # 步骤14：飞回起点上方（指挥中心 0,0），避免在窗户2位置原地降落撞框
        if not self.fly_to(0.0, 0.0, 1.0, 0.0, max_xy_speed=0.35):
            self.get_logger().warn("未返回起点上方，继续降落流程。")
        self.hover(0.5)

        # 步骤15：降落声明 → /referee/cruise_event: "landing_marker"
        self.declare_event("landing_marker")

        # 步骤16：降落（在起点上方垂直降）
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
        node.close_monitor()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
