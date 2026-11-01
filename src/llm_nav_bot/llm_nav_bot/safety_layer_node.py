"""安全層ノード: /cmd_vel_raw を LiDAR で検査してから /cmd_vel へ流す。

LLM・行動層とは独立に動き、最後に必ずここを通る。
- 前方 / 後方が近すぎる → その方向の並進を 0 (回転は許可)
- スキャンが途絶えた / 指令が途絶えた → 全停止
- /cmd_vel の型 (Twist / TwistStamped) はブリッジ側の購読者から自動判別する

時刻の判定はメッセージのヘッダ時刻ではなく「受信時のシミュレーション時刻」を使う
(Nav2 で問題になった TF / タイムスタンプ不整合の影響を受けないようにするため)。
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .core.safety import SafetyFilter, SafetyParams
from .core.scan_utils import forward_clearance, sector_distances
from .ros_common import EVENT_QOS, json_msg, sim_now

TWIST = "geometry_msgs/msg/Twist"
TWIST_STAMPED = "geometry_msgs/msg/TwistStamped"


class SafetyLayer(Node):
    def __init__(self):
        super().__init__("safety_layer")
        self.declare_parameter("output_topic", "cmd_vel")
        self.declare_parameter("cmd_vel_type", "auto")  # auto / twist / twist_stamped
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("stop_distance", 0.22)
        self.declare_parameter("slow_distance", 0.45)
        self.declare_parameter("back_stop_distance", 0.20)
        self.declare_parameter("scan_timeout", 1.0)
        self.declare_parameter("max_linear", 0.20)
        self.declare_parameter("max_angular", 1.2)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.filter = SafetyFilter(SafetyParams(
            stop_distance=float(gp("stop_distance")),
            slow_distance=float(gp("slow_distance")),
            back_stop_distance=float(gp("back_stop_distance")),
            scan_timeout=float(gp("scan_timeout")),
            max_linear=float(gp("max_linear")),
            max_angular=float(gp("max_angular")),
        ))
        self.output_topic = gp("output_topic")
        self.type_mode = gp("cmd_vel_type")
        self.frame_id = gp("frame_id")
        self.cmd_timeout = float(gp("cmd_timeout"))

        self.sectors = None
        self.scan_time = None
        self.cmd = (0.0, 0.0)
        self.cmd_time = None
        self.state = None
        self.out_pub = None
        self.out_stamped = False
        self.interventions = 0

        self.create_subscription(LaserScan, "scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(Twist, "cmd_vel_raw", self.on_cmd, 10)
        self.state_pub = self.create_publisher(String, "safety_state", EVENT_QOS)
        self.create_timer(1.0 / float(gp("rate_hz")), self.on_timer)
        self.create_timer(1.0, self.publish_state)  # 状態の定期送信 (ロガー・表示用)

    # ------------------------------------------------------------------
    def _ensure_publisher(self):
        if self.out_pub is not None:
            return True
        mode = self.type_mode
        if mode == "auto":
            types = {i.topic_type for i in self.get_subscriptions_info_by_topic(self.output_topic)}
            if TWIST_STAMPED in types:
                mode = "twist_stamped"
            elif TWIST in types:
                mode = "twist"
            else:
                self.get_logger().warn(
                    f"/{self.output_topic} の購読者 (ros_gz_bridge) が見つかりません。"
                    "ブリッジが起動しているか確認してください", throttle_duration_sec=5.0)
                return False
        self.out_stamped = mode == "twist_stamped"
        msg_type = TwistStamped if self.out_stamped else Twist
        self.out_pub = self.create_publisher(msg_type, self.output_topic, 10)
        self.get_logger().info(f"/{self.output_topic} を {msg_type.__name__} で出力します")
        return True

    def on_scan(self, msg):
        self.sectors = sector_distances(msg.ranges, msg.angle_min, msg.angle_increment,
                                        msg.range_min, msg.range_max)
        self.scan_time = sim_now(self)

    def on_cmd(self, msg):
        self.cmd = (msg.linear.x, msg.angular.z)
        self.cmd_time = sim_now(self)

    # ------------------------------------------------------------------
    def on_timer(self):
        if not self._ensure_publisher():
            return
        now = sim_now(self)
        v_in, w_in = self.cmd
        if self.cmd_time is None or now - self.cmd_time > self.cmd_timeout:
            v_in, w_in = 0.0, 0.0  # 上流が止まったら止める (watchdog)
        scan_age = None if self.scan_time is None else now - self.scan_time
        v, w, state = self.filter.filter(v_in, w_in, self.sectors, scan_age)

        if state != self.state:
            if state not in ("OK", "SLOW"):
                self.interventions += 1
                self.get_logger().warn(f"安全層が介入: {state}")
            self.state = state
            self.publish_state(v_in, w_in, v, w)

        if self.out_stamped:
            out = TwistStamped()
            out.header.stamp = self.get_clock().now().to_msg()
            out.header.frame_id = self.frame_id
            out.twist.linear.x = float(v)
            out.twist.angular.z = float(w)
        else:
            out = Twist()
            out.linear.x = float(v)
            out.angular.z = float(w)
        self.out_pub.publish(out)

    def publish_state(self, v_in=None, w_in=None, v=None, w=None):
        if self.state is None:
            return
        data = {"t": round(sim_now(self), 2), "state": self.state,
                "interventions": self.interventions}
        if self.sectors is not None:
            data["forward_clearance"] = round(forward_clearance(self.sectors), 3)
            data["back"] = round(self.sectors["back"], 3)
        if v_in is not None:
            data.update({"v_in": round(v_in, 3), "w_in": round(w_in, 3),
                         "v_out": round(v, 3), "w_out": round(w, 3)})
        self.state_pub.publish(json_msg(data))

    def stop_robot(self):
        if self.out_pub is None:
            return
        self.out_pub.publish(TwistStamped() if self.out_stamped else Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafetyLayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.try_shutdown()
