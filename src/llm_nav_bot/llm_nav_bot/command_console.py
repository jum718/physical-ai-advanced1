"""対話コンソール: 指示を入力し、LLM の解釈・計画・実行状況・安全層の介入を表示する。

デモ動画ではこの画面と Gazebo を並べて撮影すると、「指示 → AI の解釈 → 行動」が伝わりやすい。

使い方:  ros2 run llm_nav_bot console
         (q で終了)
"""

import sys
import threading

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from .ros_common import EVENT_QOS, parse_json

CYAN, GREEN, YELLOW, RED, DIM, RESET = ("\033[36m", "\033[32m", "\033[33m", "\033[31m",
                                        "\033[2m", "\033[0m")

EXAMPLES = [
    "1m進んでから左に90度回って",
    "右の壁に沿って30秒走って",
    "ぶつからないように部屋を1分探索して",
    "一番広い方向を向いて、50cm進んで",
    "0.5mの正方形を描いて",
    "止まって",
]


class Console(Node):
    def __init__(self):
        super().__init__("llm_nav_console")
        self.pub = self.create_publisher(String, "user_command", EVENT_QOS)
        self.create_subscription(String, "llm_response", self.on_response, EVENT_QOS)
        self.create_subscription(String, "plan_status", self.on_plan_status, EVENT_QOS)
        self.create_subscription(String, "safety_state", self.on_safety, EVENT_QOS)
        self.last_safety = None
        self.empty_plans = set()

    def send(self, text):
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    @staticmethod
    def out(text):
        sys.stdout.write("\r" + text + "\n指示> ")
        sys.stdout.flush()

    def on_response(self, msg):
        r = parse_json(msg)
        if not r:
            return
        lines = [f"{CYAN}[AI #{r['plan_id']} | {r['backend']} {r['latency_s']:.1f}s]{RESET}"]
        if r.get("understanding"):
            lines.append(f"  解釈: {r['understanding']}")
        steps = r.get("steps") or []
        if not steps:
            self.empty_plans.add(r["plan_id"])  # 何もしない計画には「完了」を出さない
        if steps:
            lines.append("  計画: " + "  ".join(f"{i + 1}) {s}" for i, s in enumerate(steps))
                         + (f"  ×{r['repeat']}" if r.get("repeat", 1) > 1 else ""))
        if r.get("reply"):
            lines.append(f"  {GREEN}ロボット: {r['reply']}{RESET}")
        for w in r.get("warnings") or []:
            lines.append(f"  {YELLOW}補正: {w}{RESET}")
        self.out("\n".join(lines))

    def on_plan_status(self, msg):
        ev = parse_json(msg)
        if not ev:
            return
        t = ev["type"]
        pid = ev.get("plan_id")
        if t == "step_started":
            self.out(f"{DIM}  ▶ #{pid} step {ev['index'] + 1}: {ev.get('summary')}{RESET}")
        elif t == "plan_finished":
            if pid in self.empty_plans:
                return
            self.out(f"{GREEN}  ✔ #{pid} 完了{RESET}")
        elif t == "plan_blocked":
            self.out(f"{YELLOW}  ■ #{pid} 中断: {ev.get('note')}{RESET}")
        elif t == "step_done" and ev.get("result") == "timeout":
            self.out(f"{YELLOW}  … #{pid} step {ev['index'] + 1} 時間切れ{RESET}")

    def on_safety(self, msg):
        s = parse_json(msg)
        if not s or s["state"] == self.last_safety:
            return
        self.last_safety = s["state"]
        if s["state"] in ("OK", "SLOW"):
            return
        if s["state"] == "NO_SCAN":
            detail = "(LiDAR が受信できていません)"
        else:
            detail = f"(前方 {s.get('forward_clearance', 0):.2f}m)"
        self.out(f"{RED}  ⚠ 安全層が介入: {s['state']} {detail}{RESET}")


def main(args=None):
    rclpy.init(args=args)
    node = Console()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_quietly():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass  # 終了時の rclpy.try_shutdown() による正常な停止

    spin = threading.Thread(target=spin_quietly, daemon=True)
    spin.start()

    print("自然言語でロボットに指示してください (q で終了)。例:")
    for e in EXAMPLES:
        print(f"  - {e}")
    try:
        while rclpy.ok():
            text = input("指示> ").strip()
            if text in ("q", "quit", "exit"):
                break
            if text:
                node.send(text)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        # spin スレッドが動いたままノードを破棄すると std::terminate で落ちるため、
        # 先に rclpy を終了して spin を止め、スレッドの終了を待ってから破棄する
        rclpy.try_shutdown()
        spin.join(timeout=2.0)
        executor.shutdown()
        node.destroy_node()
