"""ROS ノード間で共有する小さなヘルパー。"""

import json
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import String

# 計画・イベント系の JSON トピックは取りこぼしたくないので reliable
EVENT_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def sim_now(node):
    """ノードの時計 (use_sim_time=true なら /clock) の現在時刻 [s]。"""
    return node.get_clock().now().nanoseconds * 1e-9


def json_msg(data):
    msg = String()
    msg.data = json.dumps(data, ensure_ascii=False)
    return msg


def parse_json(msg):
    try:
        return json.loads(msg.data)
    except (ValueError, TypeError):
        return None
