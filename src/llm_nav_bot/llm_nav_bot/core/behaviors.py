"""行動ステップの実行ロジック (反応型制御)。

各ステップは観測 (Observation) を受け取り、速度指令 (Command) を返す。
自己位置は odom のみを使い、map 座標・AMCL・TF には依存しない。
時刻はシミュレーション時刻 [s] を使うので、Gazebo の RTF が低くても挙動は変わらない。
"""

import math
from dataclasses import dataclass, field

from .scan_utils import forward_clearance, normalize_rad


@dataclass
class Observation:
    t: float                 # シミュレーション時刻 [s]
    x: float                 # odom 座標 [m]
    y: float
    yaw: float               # [rad]
    sectors: dict            # scan_utils.sector_distances() の結果
    open_dir_deg: float = 0.0


@dataclass
class Command:
    v: float = 0.0           # 並進速度 [m/s]
    w: float = 0.0           # 角速度 [rad/s] (+ = 左回り)
    done: bool = False
    result: str = ""         # done 時: "ok" / "blocked" / "timeout"
    note: str = ""


@dataclass
class BehaviorParams:
    max_angular: float = 0.8          # 行動層が出す角速度の上限 [rad/s]
    min_turn_angular: float = 0.25    # 回転の最小角速度 (静止摩擦で止まらないように)
    obstacle_distance: float = 0.45   # 前進系行動が「前が塞がっている」と判断する距離 [m]
    blocked_timeout: float = 4.0      # この時間塞がったままならステップを blocked で終了 [s]
    turn_tolerance_deg: float = 3.0
    turn_kp: float = 1.5
    heading_kp: float = 1.0           # 直進時の方位保持ゲイン
    wall_kp: float = 1.5
    wall_lost_distance: float = 1.0   # 壁沿いで壁を見失ったとみなす距離 [m]


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


class StepRunner:
    """1 つの Step を実行する状態機械。"""

    def __init__(self, step, params, obs):
        self.step = step
        self.p = params
        self.t0 = obs.t
        self.x0, self.y0, self.yaw0 = obs.x, obs.y, obs.yaw
        self.last_t = obs.t
        self.last_yaw = obs.yaw
        self.rotated = 0.0          # 累積回転角 [rad] (±180° をまたぐ回転に対応)
        self.blocked_time = 0.0
        self.explore_turn_dir = 0   # explore: 0=前進中, ±1=回避旋回中
        self.explore_turn_time = 0.0
        self.explore_w = 0.0        # explore: 平滑化した直進中の舵角速度

        self.turn_target = math.radians(step.angle_deg)
        if step.action == "turn_to_open":
            self.turn_target = math.radians(obs.open_dir_deg)

        speed = max(step.speed, 0.03)
        if step.action in ("forward", "backward"):
            self.timeout = step.distance_m / speed * 3.0 + 10.0
        elif step.action in ("turn", "turn_to_open"):
            self.timeout = abs(self.turn_target) / params.min_turn_angular * 2.0 + 5.0
        else:
            self.timeout = step.duration_s + 5.0

    # ------------------------------------------------------------------
    def update(self, obs):
        dt = max(0.0, obs.t - self.last_t)
        self.last_t = obs.t
        self.rotated += normalize_rad(obs.yaw - self.last_yaw)
        self.last_yaw = obs.yaw
        elapsed = obs.t - self.t0

        if elapsed > self.timeout:
            return Command(done=True, result="timeout", note="時間切れ")

        handler = getattr(self, "_" + self.step.action)
        return handler(obs, dt, elapsed)

    # ------------------------------------------------------------------
    def _forward(self, obs, dt, elapsed):
        return self._straight(obs, dt, direction=+1)

    def _backward(self, obs, dt, elapsed):
        return self._straight(obs, dt, direction=-1)

    def _straight(self, obs, dt, direction):
        traveled = math.hypot(obs.x - self.x0, obs.y - self.y0)
        remaining = self.step.distance_m - traveled
        if remaining <= 0.01:
            return Command(done=True, result="ok")

        if direction > 0:
            clear = forward_clearance(obs.sectors)
            blocked = clear < self.p.obstacle_distance
        else:
            clear = obs.sectors["back"]
            blocked = clear < self.p.obstacle_distance * 0.7
        if blocked:
            self.blocked_time += dt
            if self.blocked_time > self.p.blocked_timeout:
                # 進行済みの距離を添える (再計画で、残りの距離を LLM が判断できるように)
                return Command(done=True, result="blocked",
                               note=f"障害物 {clear:.2f}m で進めない"
                                    f"（指示 {self.step.distance_m:.2f}m のうち "
                                    f"{traveled:.2f}m 進行済み）")
            return Command(note="障害物のため待機")
        self.blocked_time = 0.0

        # 目標手前で減速し、開始時の方位を保つ
        v = min(self.step.speed, max(0.04, remaining * 1.5))
        w = _clamp(self.p.heading_kp * normalize_rad(self.yaw0 - obs.yaw), -0.3, 0.3)
        return Command(v=direction * v, w=w)

    # ------------------------------------------------------------------
    def _turn(self, obs, dt, elapsed):
        remaining = self.turn_target - self.rotated
        if abs(remaining) < math.radians(self.p.turn_tolerance_deg):
            return Command(done=True, result="ok")
        w = self.p.turn_kp * remaining
        w = math.copysign(_clamp(abs(w), self.p.min_turn_angular, self.p.max_angular), remaining)
        return Command(w=w)

    def _turn_to_open(self, obs, dt, elapsed):
        return self._turn(obs, dt, elapsed)

    # ------------------------------------------------------------------
    def _wall_follow(self, obs, dt, elapsed):
        if elapsed >= self.step.duration_s:
            return Command(done=True, result="ok")
        s = obs.sectors
        side = self.step.side
        # 右壁なら +1 (壁に近すぎたら左へ)、左壁なら -1
        sign = 1.0 if side == "right" else -1.0
        d_side = min(s[side], s["front_" + side] * 0.7)
        clear = forward_clearance(s)

        if clear < self.p.obstacle_distance + 0.1:
            # 前方が詰まった (内角のコーナー): 壁と反対側へその場旋回
            return Command(v=0.0, w=sign * self.p.max_angular * 0.8, note="コーナー旋回")
        if d_side > self.p.wall_lost_distance:
            # 壁を見失った (外角のコーナー): 壁側へゆるく曲がって壁を探す
            return Command(v=self.step.speed * 0.7, w=-sign * 0.35, note="壁を探索")
        err = self.step.target_distance_m - d_side
        w = _clamp(sign * self.p.wall_kp * err, -self.p.max_angular * 0.6,
                   self.p.max_angular * 0.6)
        return Command(v=self.step.speed, w=w)

    # ------------------------------------------------------------------
    def _explore(self, obs, dt, elapsed):
        if elapsed >= self.step.duration_s:
            return Command(done=True, result="ok")
        s = obs.sectors
        clear = forward_clearance(s)

        if self.explore_turn_dir == 0:
            if clear < self.p.obstacle_distance:
                # 左右で空いている方へ回避旋回を開始
                left_space = s["front_left"] + s["left"]
                right_space = s["front_right"] + s["right"]
                self.explore_turn_dir = 1 if left_space >= right_space else -1
                self.explore_turn_time = 0.0
                self.explore_w = 0.0
            else:
                # Braitenberg 的に、開けている側へ少しずつ舵を切りながら前進
                bias = min(s["front_left"], 1.5) - min(s["front_right"], 1.5)
                w_target = _clamp(0.6 * bias, -0.4, 0.4)
                # 左右の距離差はスキャンごとに大きく揺れるので、1 次遅れで平滑化して
                # 舵の左右往復 (チャタリング) を抑える (時定数 約 1 秒)
                alpha = min(1.0, dt / 1.0) if dt > 0 else 0.1
                self.explore_w += alpha * (w_target - self.explore_w)
                return Command(v=self.step.speed, w=self.explore_w, note="")

        # 回避旋回中: 前が十分空いたら前進に戻る (ヒステリシス 0.15m)
        self.explore_turn_time += dt
        if clear > self.p.obstacle_distance + 0.15:
            self.explore_turn_dir = 0
            return Command(note="回避完了")
        if self.explore_turn_time > 8.0:
            # 片側に回り続けても抜けられない場合は反対へ
            self.explore_turn_dir *= -1
            self.explore_turn_time = 0.0
        return Command(w=self.explore_turn_dir * self.p.max_angular * 0.7, note="回避旋回")

    # ------------------------------------------------------------------
    def _stop(self, obs, dt, elapsed):
        if elapsed >= self.step.duration_s:
            return Command(done=True, result="ok")
        return Command()


@dataclass
class _Progress:
    index: int = 0
    repeat_index: int = 0


class PlanExecutor:
    """Plan のステップを順番に実行し、進行イベントを返す。"""

    def __init__(self, params=None):
        self.params = params or BehaviorParams()
        self.plan = None
        self.runner = None
        self.progress = _Progress()
        self.last_result = "idle"

    @property
    def active(self):
        return self.plan is not None

    @property
    def current_step(self):
        if not self.active or self.progress.index >= len(self.plan.steps):
            return None
        return self.plan.steps[self.progress.index]

    def _event(self, etype, **extra):
        ev = {"type": etype, "plan_id": self.plan.plan_id if self.plan else 0}
        ev.update(extra)
        return ev

    def start(self, plan):
        events = self.cancel("新しい計画で置き換え") if self.active else []
        self.plan = plan
        self.runner = None
        self.progress = _Progress()
        events.append(self._event("plan_started", steps=len(plan.steps), repeat=plan.repeat,
                                  summary=plan.summary()))
        if not plan.steps:
            events.append(self._event("plan_finished"))
            self.last_result = "finished"
            self.plan = None
        return events

    def cancel(self, reason=""):
        if not self.active:
            return []
        ev = [self._event("plan_cancelled", reason=reason)]
        self.plan = None
        self.runner = None
        self.last_result = "cancelled"
        return ev

    def tick(self, obs):
        if not self.active:
            return Command(), []
        events = []
        step = self.current_step
        if self.runner is None:
            self.runner = StepRunner(step, self.params, obs)
            events.append(self._event("step_started", index=self.progress.index,
                                      repeat_index=self.progress.repeat_index,
                                      action=step.action, summary=step.summary()))
        cmd = self.runner.update(obs)
        if not cmd.done:
            return cmd, events

        events.append(self._event("step_done", index=self.progress.index,
                                  action=step.action, result=cmd.result, note=cmd.note))
        self.runner = None
        if cmd.result == "blocked":
            events.append(self._event("plan_blocked", index=self.progress.index,
                                      action=step.action, note=cmd.note))
            self.last_result = "blocked"
            self.plan = None
            return Command(), events

        self.progress.index += 1
        if self.progress.index >= len(self.plan.steps):
            self.progress.index = 0
            self.progress.repeat_index += 1
            if self.progress.repeat_index >= self.plan.repeat:
                events.append(self._event("plan_finished"))
                self.last_result = "finished"
                self.plan = None
        # ステップの切り替わりでは一度停止指令を出し、次の tick で次ステップを開始する
        return Command(), events
