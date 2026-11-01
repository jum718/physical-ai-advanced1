"""LLM が出力する「行動計画」のスキーマと検証。

LLM の出力はそのまま信用せず、validate_plan() で
未知の行動の除去・数値のクランプ・デフォルト補完を行ってからロボットに渡す。
"""

import json
import math
from dataclasses import asdict, dataclass, field

ACTIONS = (
    "forward",       # 前進 distance_m
    "backward",      # 後退 distance_m
    "turn",          # その場回転 angle_deg (+左 / -右)
    "turn_to_open",  # 最も開けた方向を向く
    "wall_follow",   # 壁沿い走行 side, target_distance_m, duration_s
    "explore",       # 障害物を避けながら探索 duration_s
    "stop",          # 停止・待機 duration_s
)


@dataclass
class Step:
    action: str
    distance_m: float = 0.0
    angle_deg: float = 0.0
    duration_s: float = 0.0
    speed: float = 0.0
    side: str = "right"
    target_distance_m: float = 0.4

    def summary(self):
        """人間向けの短い説明 (ログ・コンソール表示用)。"""
        a = self.action
        if a in ("forward", "backward"):
            return f"{a} {self.distance_m:.2f}m @{self.speed:.2f}m/s"
        if a == "turn":
            return f"turn {self.angle_deg:+.0f}°"
        if a == "wall_follow":
            return (f"wall_follow {self.side} {self.target_distance_m:.2f}m "
                    f"{self.duration_s:.0f}s")
        if a == "explore":
            return f"explore {self.duration_s:.0f}s @{self.speed:.2f}m/s"
        if a == "stop":
            return f"stop {self.duration_s:.1f}s"
        return a


@dataclass
class Plan:
    steps: list = field(default_factory=list)
    repeat: int = 1
    understanding: str = ""
    reply: str = ""
    plan_id: int = 0
    instruction: str = ""

    def to_dict(self):
        d = asdict(self)
        d["steps"] = [asdict(s) for s in self.steps]
        return d

    def to_json(self):
        return json.dumps(self.to_dict(), ensure_ascii=False)

    def summary(self):
        body = " → ".join(s.summary() for s in self.steps) or "(行動なし)"
        return body + (f"  ×{self.repeat}" if self.repeat > 1 else "")


@dataclass
class Limits:
    """LLM の出力に関係なく必ず守らせる上限値 (TurtleBot3 Burger の最大は 0.22m/s)。"""
    max_linear: float = 0.18
    min_linear: float = 0.03
    default_linear: float = 0.12
    max_distance: float = 3.0
    max_duration: float = 90.0
    max_steps: int = 10
    max_repeat: int = 5
    min_wall_distance: float = 0.25
    max_wall_distance: float = 1.0


# Claude の structured outputs (output_config.format) に渡す JSON Schema。
# 使わないフィールドは null を許可し、範囲チェックは validate_plan() 側で行う。
_NUM_OR_NULL = {"anyOf": [{"type": "number"}, {"type": "null"}]}
STEP_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "action": {"type": "string", "enum": list(ACTIONS)},
        "distance_m": _NUM_OR_NULL,
        "angle_deg": _NUM_OR_NULL,
        "duration_s": _NUM_OR_NULL,
        "speed": _NUM_OR_NULL,
        "side": {"anyOf": [{"type": "string", "enum": ["left", "right"]},
                           {"type": "null"}]},
        "target_distance_m": _NUM_OR_NULL,
    },
    "required": ["action", "distance_m", "angle_deg", "duration_s", "speed",
                 "side", "target_distance_m"],
    "additionalProperties": False,
}
PLAN_JSON_SCHEMA = {
    "type": "object",
    "properties": {
        "understanding": {"type": "string"},
        "reply": {"type": "string"},
        "steps": {"type": "array", "items": STEP_JSON_SCHEMA},
        "repeat": {"type": "integer"},
    },
    "required": ["understanding", "reply", "steps", "repeat"],
    "additionalProperties": False,
}


def _num(value, default):
    try:
        f = float(value)
    except (TypeError, ValueError):
        return default
    return f if math.isfinite(f) else default


def _clamp(x, lo, hi):
    return max(lo, min(hi, x))


def _validate_step(raw, limits, warnings, index):
    if not isinstance(raw, dict):
        warnings.append(f"step{index + 1}: オブジェクトではないため無視")
        return None
    action = raw.get("action")
    if action not in ACTIONS:
        warnings.append(f"step{index + 1}: 未知の行動 '{action}' を無視")
        return None

    def clamped(key, default, lo, hi, label):
        v = _num(raw.get(key), default)
        c = _clamp(v, lo, hi)
        if abs(c - v) > 1e-9:
            warnings.append(f"step{index + 1}: {label} {v:g} → {c:g} に制限")
        return c

    step = Step(action=action)
    if action in ("forward", "backward", "wall_follow", "explore"):
        # 速度は大きさだけ使い、向きは action で決める
        speed = abs(_num(raw.get("speed"), limits.default_linear))
        step.speed = _clamp(speed, limits.min_linear, limits.max_linear)
        if abs(step.speed - speed) > 1e-9:
            warnings.append(f"step{index + 1}: speed {speed:g} → {step.speed:g} に制限")

    if action in ("forward", "backward"):
        step.distance_m = clamped("distance_m", 0.5, 0.05, limits.max_distance, "distance_m")
    elif action == "turn":
        step.angle_deg = clamped("angle_deg", 90.0, -360.0, 360.0, "angle_deg")
        if abs(step.angle_deg) < 1.0:
            warnings.append(f"step{index + 1}: 回転角がほぼ 0 のため無視")
            return None
    elif action == "wall_follow":
        side = raw.get("side")
        step.side = side if side in ("left", "right") else "right"
        step.target_distance_m = clamped("target_distance_m", 0.4, limits.min_wall_distance,
                                         limits.max_wall_distance, "target_distance_m")
        step.duration_s = clamped("duration_s", 20.0, 1.0, limits.max_duration, "duration_s")
    elif action == "explore":
        step.duration_s = clamped("duration_s", 30.0, 1.0, limits.max_duration, "duration_s")
    elif action == "stop":
        step.duration_s = clamped("duration_s", 1.0, 0.0, limits.max_duration, "duration_s")
    return step


def validate_plan(raw, limits=None):
    """LLM の生出力 (dict) を検証済みの Plan に変換する。

    戻り値: (Plan, warnings: list[str])
    """
    limits = limits or Limits()
    warnings = []
    if not isinstance(raw, dict):
        return Plan(reply="計画を解釈できませんでした。"), ["計画が JSON オブジェクトではありません"]

    raw_steps = raw.get("steps") or []
    if not isinstance(raw_steps, list):
        warnings.append("steps が配列ではないため無視")
        raw_steps = []
    if len(raw_steps) > limits.max_steps:
        warnings.append(f"ステップ数 {len(raw_steps)} → {limits.max_steps} に制限")
        raw_steps = raw_steps[:limits.max_steps]

    steps = []
    for i, rs in enumerate(raw_steps):
        s = _validate_step(rs, limits, warnings, i)
        if s is not None:
            steps.append(s)

    repeat = int(_clamp(_num(raw.get("repeat"), 1), 1, limits.max_repeat))
    plan = Plan(
        steps=steps,
        repeat=repeat,
        understanding=str(raw.get("understanding") or ""),
        reply=str(raw.get("reply") or ""),
        plan_id=int(_num(raw.get("plan_id"), 0)),
        instruction=str(raw.get("instruction") or ""),
    )
    return plan, warnings


def stop_plan(reply="停止します。"):
    return Plan(steps=[Step(action="stop", duration_s=0.0)], understanding="緊急停止",
                reply=reply)
