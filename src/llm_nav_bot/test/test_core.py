"""コアロジックの単体テスト (ROS 不要)。

実行: cd src/llm_nav_bot && python -m pytest test -q
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.dirname(__file__))

from llm_nav_bot.core.plan_schema import Plan, Step, validate_plan  # noqa: E402
from llm_nav_bot.core.planners import (REPLAN_PREFIX, RulePlanner,  # noqa: E402
                                       format_user_message, is_stop_command)
from llm_nav_bot.core.safety import NO_SCAN, STOP_FRONT, SafetyFilter  # noqa: E402
from llm_nav_bot.core.scan_utils import (most_open_direction,  # noqa: E402
                                         normalize_deg, sector_distances)
from sim2d import ROBOT_RADIUS, Sim, World, run  # noqa: E402

PILLARS = [(1.0, 0.0, 0.15), (-0.8, 1.0, 0.15), (0.3, -1.2, 0.2), (-1.0, -0.9, 0.15),
           (1.1, 1.2, 0.15)]


def _plan(*steps, repeat=1):
    plan, warnings = validate_plan({"steps": list(steps), "repeat": repeat})
    assert not warnings, warnings
    return plan


# ---------------------------------------------------------------- scan_utils
def test_normalize_deg():
    assert normalize_deg(190) == -170
    assert normalize_deg(-180) == 180
    assert normalize_deg(360) == 0


def test_sector_distances_wrap_and_invalid():
    n = 360
    inc = 2 * math.pi / n
    ranges = [float("inf")] * n
    ranges[0] = 0.8          # 正面
    ranges[90] = 0.5         # 左
    ranges[180] = 0.3        # 後ろ (±180° をまたぐセクター)
    ranges[270] = float("nan")
    ranges[300] = 0.05       # range_min 未満 → 無視
    s = sector_distances(ranges, 0.0, inc, 0.12, 3.5)
    assert s["front"] == 0.8
    assert s["left"] == 0.5
    assert s["back"] == 0.3
    assert s["right"] == 3.5
    assert s["front_right"] == 3.5


def test_most_open_direction_corridor():
    # 左右が壁、前が開けている → 正面付近
    n, inc = 360, 2 * math.pi / 360
    ranges = []
    for i in range(n):
        a = normalize_deg(i)
        ranges.append(0.4 if 60 <= abs(a) <= 120 else (0.6 if abs(a) > 120 else 3.0))
    angle, _ = most_open_direction(ranges, 0.0, inc, 0.12, 3.5)
    assert abs(angle) <= 20


def _scan_from(fn, range_max=3.5):
    """角度 [deg] → 距離 を返す関数から 360 本の LiDAR を作る。"""
    inc = 2 * math.pi / 360
    ranges = []
    for i in range(360):
        r = fn(normalize_deg(i))
        ranges.append(float("inf") if r is None or r >= range_max else r)
    return sector_distances(ranges, 0.0, inc, 0.12, range_max)


def test_forward_clearance_ignores_wall_beside_robot():
    """壁のすぐ横 (0.30m) を通るだけでは「前が塞がっている」と判定しない。
    (Gazebo で、前方 1.4m が空いているのに前進が中断された不具合の再発防止)"""
    from llm_nav_bot.core.scan_utils import forward_clearance

    def right_wall(a):                      # 右 0.30m に平行な壁
        return 0.30 / abs(math.sin(math.radians(a))) if -179 < a < -1 else None

    s = _scan_from(right_wall)
    assert s["right"] < 0.35 and s["front_right"] < 0.5   # 壁は確かに近くにある
    # 正面 ±20° の端が壁を斜めに捉える (約 0.88m) が、停止のしきい値 0.45m を十分に上回る
    assert forward_clearance(s) > 0.7
    assert s["corridor"] > 3.0                            # 進路の幅の中には何もない


def test_forward_clearance_detects_wall_cutting_into_path():
    from llm_nav_bot.core.scan_utils import forward_clearance

    def tilted_wall(a):                     # 壁 y = -0.30 + 0.35x (進路に食い込む)
        t = math.tan(math.radians(a))
        if math.cos(math.radians(a)) <= 0 or t >= 0.35:
            return None
        return 0.30 / (0.35 - t) / math.cos(math.radians(a))

    clear = forward_clearance(_scan_from(tilted_wall))
    assert 0.25 < clear < 0.42, clear       # 進路に入る点 (x ≈ 0.34m) を検出する


def test_forward_clearance_detects_obstacle_ahead():
    from llm_nav_bot.core.scan_utils import forward_clearance

    def pillar(a):                          # 正面 0.5m、幅 ±0.1m の柱
        if abs(a) < 90 and abs(0.5 * math.tan(math.radians(a))) < 0.1:
            return 0.5 / math.cos(math.radians(a))
        return None

    assert abs(forward_clearance(_scan_from(pillar)) - 0.5) < 0.03


# ---------------------------------------------------------------- plan_schema
def test_validate_plan_clamps_and_filters():
    raw = {
        "understanding": "x", "reply": "y", "repeat": 99,
        "steps": [
            {"action": "forward", "distance_m": 50, "speed": 5.0},
            {"action": "fly"},
            {"action": "turn", "angle_deg": 0.2},
            {"action": "wall_follow", "side": "up", "target_distance_m": 0.01, "duration_s": None},
            {"action": "backward", "distance_m": "abc", "speed": -0.1},
        ],
    }
    plan, warnings = validate_plan(raw)
    assert [s.action for s in plan.steps] == ["forward", "wall_follow", "backward"]
    f, w, b = plan.steps
    assert f.distance_m == 3.0 and f.speed == 0.18
    assert w.side == "right" and w.target_distance_m == 0.25 and w.duration_s == 20.0
    assert b.distance_m == 0.5 and b.speed == 0.1
    assert plan.repeat == 5
    assert any("fly" in m for m in warnings)


def test_validate_plan_not_dict():
    plan, warnings = validate_plan("hello")
    assert plan.steps == [] and warnings


# ---------------------------------------------------------------- planners
def test_rule_planner_sequence():
    raw = RulePlanner().plan("1m進んで、右に90度回って、それから右の壁沿いに20秒走って")
    plan, _ = validate_plan(raw)
    acts = [(s.action, round(s.distance_m, 2), s.angle_deg, s.side) for s in plan.steps]
    assert acts[0][:2] == ("forward", 1.0)
    assert acts[1][0] == "turn" and acts[1][2] == -90.0
    assert acts[2][0] == "wall_follow" and acts[2][3] == "right"
    assert plan.steps[2].duration_s == 20.0


def test_rule_planner_misc():
    p, _ = validate_plan(RulePlanner().plan("30cm下がって、一番広い方を向いて、左に45度"))
    assert [s.action for s in p.steps] == ["backward", "turn_to_open", "turn"]
    assert abs(p.steps[0].distance_m - 0.3) < 1e-9 and p.steps[2].angle_deg == 45.0
    p, _ = validate_plan(RulePlanner().plan("ゆっくり部屋を60秒探索して"))
    assert p.steps[0].action == "explore" and p.steps[0].speed == 0.06
    p, _ = validate_plan(RulePlanner().plan(REPLAN_PREFIX + "前へ"))
    assert [s.action for s in p.steps] == ["backward", "turn_to_open"]
    p, _ = validate_plan(RulePlanner().plan("空を飛んで"))
    assert p.steps == []


def test_rule_planner_splits_dekara():
    """「進んでから回って」のように、で/てから で続く指示を 2 つの行動に分ける。"""
    p, _ = validate_plan(RulePlanner().plan("1m進んでから左に90度回って"))
    assert [s.action for s in p.steps] == ["forward", "turn"]
    assert p.steps[0].distance_m == 1.0 and p.steps[1].angle_deg == 90.0


def test_is_stop_command():
    assert is_stop_command("止まって")
    assert is_stop_command("STOP")
    assert not is_stop_command("壁沿いに進んで、障害物の手前で止まって")


def test_format_user_message():
    status = {"sectors": {"front": 0.4, "left": 1.0, "right": 2.0, "back": 3.0,
                          "front_left": 1.0, "front_right": 1.0},
              "open_dir_deg": -90, "pose": {"x": 0, "y": 0, "yaw_deg": 0}}
    history = [{"instruction": "前へ", "summary": "forward 0.50m", "result": "finished"}]
    msg = format_user_message("右へ", status, history)
    assert "前 0.40m" in msg and "-90°" in msg and "前へ" in msg and msg.endswith("右へ")


def test_claude_planner_cleans_api_key(monkeypatch=None):
    """キー末尾の改行は取り除かれ、ASCII 以外 (日本語のプレースホルダ) は明示的に拒否される。"""
    try:
        import anthropic  # noqa: F401
    except ImportError:
        return  # anthropic がない環境ではスキップ
    from llm_nav_bot.core.planners import ClaudePlanner, PlannerError
    old = os.environ.get("ANTHROPIC_API_KEY")
    try:
        os.environ["ANTHROPIC_API_KEY"] = "  \"sk-ant-abc\"\r\n"
        assert ClaudePlanner().client.api_key == "sk-ant-abc"
        os.environ["ANTHROPIC_API_KEY"] = "ここにキー"
        try:
            ClaudePlanner()
            raise AssertionError("PlannerError が出るはず")
        except PlannerError:
            pass
    finally:
        if old is None:
            os.environ.pop("ANTHROPIC_API_KEY", None)
        else:
            os.environ["ANTHROPIC_API_KEY"] = old


# ---------------------------------------------------------------- safety
def test_safety_filter():
    sf = SafetyFilter()
    s = {"front": 0.15, "front_wide": 0.15, "back": 2.0}
    v, w, st = sf.filter(0.15, 0.5, s, 0.1)
    assert v == 0.0 and w == 0.5 and st == STOP_FRONT  # 回転は許可
    v, w, st = sf.filter(-0.1, 0.0, s, 0.1)
    assert v == -0.1                                    # 後退は許可
    v, w, st = sf.filter(0.1, 0.1, s, 5.0)
    assert (v, w, st) == (0.0, 0.0, NO_SCAN)


# ---------------------------------------------------------------- 2D シミュレーション
def test_forward_open_space():
    r = run(Sim(World(half=3.0)), _plan({"action": "forward", "distance_m": 1.0}))
    assert r["types"][-1] == "plan_finished"
    assert abs(r["path_len"] - 1.0) < 0.05


def test_turn_accuracy():
    for angle in (90, -90, 270):
        sim = Sim(World(half=3.0))
        r = run(sim, _plan({"action": "turn", "angle_deg": angle}))
        assert r["types"][-1] == "plan_finished"
        err = normalize_deg(math.degrees(sim.yaw) - angle)
        assert abs(err) < 5, (angle, err)


def test_forward_into_wall_is_blocked_without_collision():
    sim = Sim(World(half=1.5))
    r = run(sim, _plan({"action": "forward", "distance_m": 3.0}))
    assert "plan_blocked" in r["types"]
    assert r["min_clearance"] > ROBOT_RADIUS + 0.05
    # 中断の通知に「指示の距離」と「進行済みの距離」が入る (LLM が残りの距離を判断できる)
    note = next(e["note"] for e in r["events"] if e["type"] == "plan_blocked")
    assert "指示 3.00m のうち" in note and "進行済み" in note


def test_safety_layer_alone_prevents_collision():
    """行動層が壊れて「全速前進」を出し続けても、安全層だけで止まる。"""
    from llm_nav_bot.core.safety import SafetyFilter
    sim = Sim(World(half=1.5))
    sf = SafetyFilter()
    for _ in range(400):
        sim.observe()
        v, w, _ = sf.filter(0.2, 0.0, sim.sectors, sim.t - sim.scan_time)
        sim.step(v, w)
    assert sim.world.clearance(sim.x, sim.y) > ROBOT_RADIUS


def test_explore_no_collision():
    sim = Sim(World(half=2.0, circles=PILLARS), x=-1.5, y=0.2)
    r = run(sim, _plan({"action": "explore", "duration_s": 90, "speed": 0.15}), max_time=120)
    assert r["types"][-1] == "plan_finished"
    assert r["min_clearance"] > ROBOT_RADIUS, r["min_clearance"]
    assert r["path_len"] > 4.0


def test_wall_follow_keeps_distance():
    sim = Sim(World(half=2.0), x=0.0, y=-1.5, yaw=0.0)   # 右側 (y=-2) に壁
    r = run(sim, _plan({"action": "wall_follow", "side": "right", "target_distance_m": 0.4,
                        "duration_s": 90}), max_time=120)
    assert r["types"][-1] == "plan_finished"
    assert r["min_clearance"] > ROBOT_RADIUS + 0.1, r["min_clearance"]
    # 部屋の外周を回るので、ほとんどの時間、壁との距離は目標付近にあるはず
    near = [sim.world.clearance(x, y) for _, x, y, _ in r["trace"][200:]]
    good = sum(1 for d in near if 0.25 < d < 0.6) / len(near)
    assert good > 0.8, good
    assert r["path_len"] > 6.0


def test_new_plan_replaces_running_one():
    from llm_nav_bot.core.behaviors import PlanExecutor
    sim = Sim(World(half=3.0))
    ex = PlanExecutor()
    ex.start(_plan({"action": "explore", "duration_s": 60}))
    for _ in range(20):
        ex.tick(sim.observe())
    events = ex.start(_plan({"action": "stop", "duration_s": 0}))
    assert [e["type"] for e in events] == ["plan_cancelled", "plan_started"]


def test_repeat_square():
    sim = Sim(World(half=3.0))
    plan = _plan({"action": "forward", "distance_m": 0.5}, {"action": "turn", "angle_deg": 90},
                 repeat=4)
    r = run(sim, plan, max_time=300)
    assert r["types"][-1] == "plan_finished"
    assert math.hypot(sim.x, sim.y) < 0.15   # 正方形を一周して元の位置付近に戻る
    assert sum(1 for t in r["types"] if t == "step_done") == 8


def test_plan_roundtrip_json():
    import json
    p = Plan(steps=[Step("forward", distance_m=1.0, speed=0.1)], reply="ok", plan_id=3)
    q, w = validate_plan(json.loads(p.to_json()))
    assert not w and q.plan_id == 3 and q.steps[0].distance_m == 1.0
