"""行動層ノード: /behavior_plan を受け取り、LiDAR と odom を見ながら実行する。

subscribe: /scan, /odom, /behavior_plan (JSON)
publish:   /cmd_vel_raw (Twist, 安全層へ), /robot_status (JSON), /plan_status (JSON イベント)
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .core.behaviors import BehaviorParams, Observation, PlanExecutor
from .core.plan_schema import Limits, validate_plan
from .core.scan_utils import forward_clearance, most_open_direction, sector_distances
from .ros_common import EVENT_QOS, json_msg, parse_json, sim_now, yaw_from_quaternion


class BehaviorController(Node):
    def __init__(self):
        super().__init__("behavior_controller")
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("status_rate_hz", 2.0)
        self.declare_parameter("max_angular", 0.8)
        self.declare_parameter("obstacle_distance", 0.45)
        self.declare_parameter("blocked_timeout", 4.0)
        self.declare_parameter("max_linear", 0.18)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.limits = Limits(max_linear=float(gp("max_linear")))
        self.runner = PlanExecutor(BehaviorParams(
            max_angular=float(gp("max_angular")),
            obstacle_distance=float(gp("obstacle_distance")),
            blocked_timeout=float(gp("blocked_timeout")),
        ))

        self.sectors = None
        self.open_dir = 0.0
        self.pose = None  # (x, y, yaw)
        self.idle_ticks = 0

        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "behavior_plan", self.on_plan, EVENT_QOS)
        self.cmd_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)
        self.status_pub = self.create_publisher(String, "robot_status", 10)
        self.event_pub = self.create_publisher(String, "plan_status", EVENT_QOS)

        self.create_timer(1.0 / float(gp("control_rate_hz")), self.on_control)
        self.create_timer(1.0 / float(gp("status_rate_hz")), self.publish_status)
        self.get_logger().info("行動層を起動しました (/behavior_plan 待ち)")

    # ------------------------------------------------------------------
    def on_scan(self, msg):
        self.sectors = sector_distances(msg.ranges, msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max)
        self.open_dir, _ = most_open_direction(msg.ranges, msg.angle_min, msg.angle_increment,
                                               msg.range_min, msg.range_max)

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quaternion(p.orientation))

    def on_plan(self, msg):
        data = parse_json(msg)
        plan, warnings = validate_plan(data, self.limits)  # 行動層でも再検証 (多重防御)
        for w in warnings:
            self.get_logger().warn(f"計画の補正: {w}")
        self.get_logger().info(f"計画 #{plan.plan_id} を受信: {plan.summary()}")
        self.emit(self.runner.start(plan))

    # ------------------------------------------------------------------
    def on_control(self):
        t = sim_now(self)
        if self.sectors is None or self.pose is None or t <= 0.0:
            return
        if not self.runner.active:
            # 待機中は指令を出さない (計画終了直後だけ停止指令を数回送る)。
            # 安全層の watchdog が「指令なし = 停止」を保証するので、
            # 待機中に /cmd_vel_raw へ手動で指令を送って安全層だけを試すこともできる。
            if self.idle_ticks < 3:
                self.cmd_pub.publish(Twist())
                self.idle_ticks += 1
            return
        self.idle_ticks = 0
        x, y, yaw = self.pose
        obs = Observation(t=t, x=x, y=y, yaw=yaw, sectors=self.sectors,
                          open_dir_deg=self.open_dir)
        cmd, events = self.runner.tick(obs)
        self.emit(events)
        twist = Twist()
        twist.linear.x = float(cmd.v)
        twist.angular.z = float(cmd.w)
        self.cmd_pub.publish(twist)

    def emit(self, events):
        for ev in events:
            ev["t"] = round(sim_now(self), 2)
            self.event_pub.publish(json_msg(ev))
            detail = ev.get("summary") or ev.get("note") or ev.get("result") or ""
            self.get_logger().info(f"[{ev['type']}] #{ev.get('plan_id')} {detail}")

    def publish_status(self):
        if self.sectors is None:
            return
        step = self.runner.current_step
        status = {
            "t": round(sim_now(self), 2),
            "sectors": {k: round(v, 3) for k, v in self.sectors.items()},
            "forward_clearance": round(forward_clearance(self.sectors), 3),
            "open_dir_deg": self.open_dir,
            "active": self.runner.active,
            "action": step.action if step else "idle",
            "step_index": self.runner.progress.index if step else -1,
            "plan_id": self.runner.plan.plan_id if self.runner.active else 0,
            "last_result": self.runner.last_result,
        }
        if self.pose:
            x, y, yaw = self.pose
            status["pose"] = {"x": round(x, 3), "y": round(y, 3),
                              "yaw_deg": round(math.degrees(yaw), 1)}
        self.status_pub.publish(json_msg(status))


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.try_shutdown()
