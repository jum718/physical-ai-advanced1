"""LLM 指令ノード: 自然言語の指示を行動計画に変換して /behavior_plan に送る。

subscribe: /user_command (String), /robot_status (JSON), /plan_status (JSON)
publish:   /behavior_plan (JSON), /llm_response (JSON, 表示・ログ用)

- 「止まって」などの停止指示は LLM を通さず即座に停止計画を送る (遅延ゼロ)
- LLM 呼び出しは別スレッドで行い、ROS のコールバックを止めない
- 計画が障害物で中断 (plan_blocked) されたら、状況を添えて LLM に再計画させる
- Claude API が使えない場合はルールベースのプランナーへ自動で切り替える
"""

import copy
import queue
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .core.plan_schema import Limits, stop_plan, validate_plan
from .core.planners import (REPLAN_PREFIX, ClaudePlanner, PlannerError, RulePlanner,
                            is_stop_command)
from .ros_common import EVENT_QOS, json_msg, parse_json


class LlmCommander(Node):
    def __init__(self):
        super().__init__("llm_commander")
        self.declare_parameter("backend", "claude")          # claude / rule
        self.declare_parameter("model", "claude-opus-5-5")
        self.declare_parameter("effort", "low")
        self.declare_parameter("request_timeout", 30.0)
        self.declare_parameter("use_refusal_fallback", True)
        self.declare_parameter("history_size", 5)
        self.declare_parameter("auto_replan", True)
        self.declare_parameter("max_replans", 2)
        self.declare_parameter("max_linear", 0.18)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self.limits = Limits(max_linear=float(gp("max_linear")))
        self.history_size = int(gp("history_size"))
        self.auto_replan = bool(gp("auto_replan"))
        self.max_replans = int(gp("max_replans"))

        self.rule_planner = RulePlanner()
        self.planner = self.rule_planner
        if gp("backend") == "claude":
            try:
                self.planner = ClaudePlanner(model=gp("model"), effort=gp("effort"),
                                             timeout_s=float(gp("request_timeout")),
                                             use_fallback=bool(gp("use_refusal_fallback")))
                self.get_logger().info(f"プランナー: Claude ({gp('model')}, effort={gp('effort')})")
            except PlannerError as e:
                self.get_logger().error(f"Claude を初期化できません: {e} → ルールベースで動作します")
        if self.planner is self.rule_planner:
            self.get_logger().info("プランナー: ルールベース")

        self.lock = threading.Lock()
        self.status = None
        self.history = []          # [{plan_id, instruction, summary, result}]
        self.next_plan_id = 1
        self.current = None        # 実行中の計画の {plan_id, instruction, root_instruction}
        self.replans = 0
        self.jobs = queue.Queue()

        self.create_subscription(String, "user_command", self.on_command, EVENT_QOS)
        self.create_subscription(String, "robot_status", self.on_status, 10)
        self.create_subscription(String, "plan_status", self.on_plan_status, EVENT_QOS)
        self.plan_pub = self.create_publisher(String, "behavior_plan", EVENT_QOS)
        self.resp_pub = self.create_publisher(String, "llm_response", EVENT_QOS)

        self.worker = threading.Thread(target=self.work_loop, daemon=True)
        self.worker.start()
        self.get_logger().info("指示待ち: /user_command (例: ros2 run llm_nav_bot console)")

    # ------------------------------------------------------------------ callbacks
    def on_status(self, msg):
        data = parse_json(msg)
        if data:
            with self.lock:
                self.status = data

    def on_command(self, msg):
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"指示: {text}")
        if is_stop_command(text):
            # 停止は LLM を待たずに即実行。待ち行列の古い指示も捨てる
            self._drain_jobs()
            plan = stop_plan()
            self._publish_plan(plan, text, text, backend="fast-path", latency=0.0, warnings=[])
            return
        with self.lock:
            self.replans = 0
        self.jobs.put({"instruction": text, "root": text})

    def on_plan_status(self, msg):
        ev = parse_json(msg)
        if not ev:
            return
        etype = ev.get("type")
        if etype not in ("plan_finished", "plan_blocked", "plan_cancelled"):
            return
        result = {"plan_finished": "完了", "plan_blocked": "障害物で中断",
                  "plan_cancelled": "取り消し"}[etype]
        with self.lock:
            for h in self.history:
                if h["plan_id"] == ev.get("plan_id"):
                    h["result"] = result
            current = self.current
            if not current or current["plan_id"] != ev.get("plan_id"):
                return
            self.current = None
            if etype != "plan_blocked" or not self.auto_replan:
                return
            if self.replans >= self.max_replans:
                self.get_logger().warn("再計画の上限に達したため停止します")
                return
            self.replans += 1
            n = self.replans
        note = ev.get("note", "")
        instruction = (f"{REPLAN_PREFIX}直前の計画はステップ{ev.get('index', 0) + 1}"
                       f"「{ev.get('action')}」の途中で中断された（{note}）。"
                       f"元の指示「{current['root']}」を、現在のセンサー状態を踏まえて安全に"
                       f"達成する回復計画を立てて。")
        self.get_logger().info(f"自動再計画 {n}/{self.max_replans}")
        self.jobs.put({"instruction": instruction, "root": current["root"]})

    # ------------------------------------------------------------------ worker
    def _drain_jobs(self):
        try:
            while True:
                self.jobs.get_nowait()
        except queue.Empty:
            pass

    def work_loop(self):
        while rclpy.ok():
            try:
                job = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            with self.lock:
                status = copy.deepcopy(self.status)
                history = [dict(h) for h in self.history[-self.history_size:]]
            t0 = time.monotonic()
            backend = self.planner.name
            try:
                raw = self.planner.plan(job["instruction"], status, history)
            except PlannerError as e:
                self.get_logger().error(f"{backend} で計画できません: {e} → ルールベースで代替")
                raw = self.rule_planner.plan(job["instruction"], status, history)
                backend = "rule(fallback)"
            except Exception as e:  # noqa: BLE001 - 想定外の失敗でもノードは落とさない
                self.get_logger().error(f"プランナーで予期しないエラー: {e!r}")
                raw = self.rule_planner.plan(job["instruction"], status, history)
                backend = "rule(fallback)"
            latency = time.monotonic() - t0
            if not self.jobs.empty() and not job["instruction"].startswith(REPLAN_PREFIX):
                # 考えている間に新しい指示が来た → 古い計画は送らない
                self.get_logger().info("新しい指示が届いたため、この計画は破棄します")
                continue
            plan, warnings = validate_plan(raw, self.limits)
            self._publish_plan(plan, job["instruction"], job["root"], backend, latency, warnings)

    def _publish_plan(self, plan, instruction, root, backend, latency, warnings):
        with self.lock:
            plan.plan_id = self.next_plan_id
            self.next_plan_id += 1
            plan.instruction = instruction
            self.current = {"plan_id": plan.plan_id, "root": root}
            self.history.append({"plan_id": plan.plan_id, "instruction": instruction,
                                 "summary": plan.summary(), "result": "実行中"})
            self.history = self.history[-20:]
        # 先に画面表示用の返答を送り、そのあとに計画を送る
        # (コンソールで「解釈 → 実行開始」の順に表示され、空の計画の「完了」表示の判定も確実になる)
        self.resp_pub.publish(json_msg({
            "plan_id": plan.plan_id,
            "instruction": instruction,
            "backend": backend,
            "latency_s": round(latency, 2),
            "understanding": plan.understanding,
            "reply": plan.reply,
            "summary": plan.summary(),
            "steps": [s.summary() for s in plan.steps],
            "repeat": plan.repeat,
            "warnings": warnings,
        }))
        self.plan_pub.publish(json_msg(plan.to_dict()))
        self.get_logger().info(f"計画 #{plan.plan_id} [{backend} {latency:.1f}s] "
                               f"{plan.summary()} / {plan.understanding}")
        for w in warnings:
            self.get_logger().warn(f"LLM 出力の補正: {w}")


def main(args=None):
    rclpy.init(args=args)
    node = LlmCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
