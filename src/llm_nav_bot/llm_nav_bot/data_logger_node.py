"""走行ログを CSV に記録するノード (レポート用の定量評価)。

<log_dir>/<日時>/
  trajectory.csv : 一定周期の位置・速度・各方向の距離・行動・安全層の状態
  events.csv     : 指示・LLM の計画・計画の進行・安全層の介入
グラフ化: ros2 run llm_nav_bot plot_run (または python3 -m llm_nav_bot.tools.plot_run <dir>)
"""

import csv
import json
import math
import os
from datetime import datetime

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from .ros_common import EVENT_QOS, parse_json, sim_now, yaw_from_quaternion

TRAJ_FIELDS = ["t", "x", "y", "yaw_deg", "v", "w", "front", "front_left", "left",
               "front_right", "right", "back", "forward_clearance", "plan_id", "action",
               "safety_state"]
EVENT_FIELDS = ["t", "source", "type", "plan_id", "detail"]


class DataLogger(Node):
    def __init__(self):
        super().__init__("data_logger")
        self.declare_parameter("log_dir", "~/llm_nav_logs")
        self.declare_parameter("rate_hz", 5.0)
        base = os.path.expanduser(self.get_parameter("log_dir").value)
        self.run_dir = os.path.join(base, datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.run_dir, exist_ok=True)

        self.traj_file = open(os.path.join(self.run_dir, "trajectory.csv"), "w", newline="",
                              encoding="utf-8")
        self.event_file = open(os.path.join(self.run_dir, "events.csv"), "w", newline="",
                               encoding="utf-8")
        self.traj = csv.DictWriter(self.traj_file, fieldnames=TRAJ_FIELDS)
        self.events = csv.DictWriter(self.event_file, fieldnames=EVENT_FIELDS)
        self.traj.writeheader()
        self.events.writeheader()

        self.odom = None
        self.status = {}
        self.safety_state = ""

        self.create_subscription(Odometry, "odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(String, "robot_status", self.on_status, 10)
        self.create_subscription(String, "safety_state", self.on_safety, EVENT_QOS)
        self.create_subscription(String, "llm_response", self.on_llm, EVENT_QOS)
        self.create_subscription(String, "plan_status", self.on_plan_status, EVENT_QOS)
        self.create_subscription(String, "user_command", self.on_user, EVENT_QOS)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.on_timer)
        self.get_logger().info(f"ログ出力先: {self.run_dir}")

    # ------------------------------------------------------------------
    def on_odom(self, msg):
        self.odom = msg

    def on_status(self, msg):
        self.status = parse_json(msg) or self.status

    def on_safety(self, msg):
        s = parse_json(msg)
        if not s:
            return
        if s["state"] != self.safety_state:
            self.event("safety", s["state"], 0, json.dumps(s, ensure_ascii=False))
        self.safety_state = s["state"]

    def on_llm(self, msg):
        r = parse_json(msg)
        if r:
            self.event("llm", r.get("backend", ""), r.get("plan_id", 0),
                       json.dumps(r, ensure_ascii=False))

    def on_plan_status(self, msg):
        ev = parse_json(msg)
        if ev:
            self.event("plan", ev.get("type", ""), ev.get("plan_id", 0),
                       json.dumps(ev, ensure_ascii=False))

    def on_user(self, msg):
        self.event("user", "command", 0, msg.data)

    def event(self, source, etype, plan_id, detail):
        self.events.writerow({"t": round(sim_now(self), 2), "source": source, "type": etype,
                              "plan_id": plan_id, "detail": detail})
        self.event_file.flush()

    # ------------------------------------------------------------------
    def on_timer(self):
        if self.odom is None:
            return
        p = self.odom.pose.pose
        tw = self.odom.twist.twist
        sectors = self.status.get("sectors", {})
        row = {
            "t": round(sim_now(self), 2),
            "x": round(p.position.x, 4),
            "y": round(p.position.y, 4),
            "yaw_deg": round(math.degrees(yaw_from_quaternion(p.orientation)), 2),
            "v": round(tw.linear.x, 4),
            "w": round(tw.angular.z, 4),
            "forward_clearance": self.status.get("forward_clearance", ""),
            "plan_id": self.status.get("plan_id", 0),
            "action": self.status.get("action", ""),
            "safety_state": self.safety_state,
        }
        for k in ("front", "front_left", "left", "front_right", "right", "back"):
            row[k] = sectors.get(k, "")
        self.traj.writerow(row)
        self.traj_file.flush()

    def close(self):
        self.traj_file.close()
        self.event_file.close()


def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()
