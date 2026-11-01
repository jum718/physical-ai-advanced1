"""テスト用の簡易 2D シミュレータ (Gazebo なしでコアロジックを検証する)。

正方形の部屋 + 円柱障害物の中で、差動二輪ロボットを積分し、360 本の LiDAR を模擬する。
"""

import math

from llm_nav_bot.core.behaviors import Observation, PlanExecutor
from llm_nav_bot.core.safety import SafetyFilter
from llm_nav_bot.core.scan_utils import most_open_direction, sector_distances

ROBOT_RADIUS = 0.105  # TurtleBot3 Burger の外接円の半径 [m] 程度


class World:
    def __init__(self, half=2.0, circles=()):
        self.half = half
        self.circles = list(circles)

    def raycast(self, x, y, theta, max_r):
        dx, dy = math.cos(theta), math.sin(theta)
        best = max_r
        h = self.half
        for d, p in ((dx, x), (dy, y)):
            if abs(d) > 1e-9:
                t = ((h if d > 0 else -h) - p) / d
                if 0 < t < best:
                    best = t
        for cx, cy, r in self.circles:
            fx, fy = x - cx, y - cy
            b = fx * dx + fy * dy
            c = fx * fx + fy * fy - r * r
            disc = b * b - c
            if disc >= 0:
                t = -b - math.sqrt(disc)
                if 0 < t < best:
                    best = t
        return best

    def clearance(self, x, y):
        """ロボット中心から最も近い障害物表面までの距離。"""
        h = self.half
        d = min(h - x, x + h, h - y, y + h)
        for cx, cy, r in self.circles:
            d = min(d, math.hypot(x - cx, y - cy) - r)
        return d


class Sim:
    def __init__(self, world, x=0.0, y=0.0, yaw=0.0, dt=0.05, scan_period=0.2):
        self.world = world
        self.x, self.y, self.yaw = x, y, yaw
        self.t = 0.0
        self.dt = dt
        self.scan_period = scan_period
        self._next_scan = 0.0
        self.sectors = None
        self.open_dir = 0.0
        self.scan_time = None

    def _scan(self):
        n, max_r, min_r = 360, 3.5, 0.12
        inc = 2 * math.pi / n
        ranges = []
        for i in range(n):
            r = self.world.raycast(self.x, self.y, self.yaw + i * inc, max_r)
            ranges.append(float("inf") if r >= max_r else r)
        self.sectors = sector_distances(ranges, 0.0, inc, min_r, max_r)
        self.open_dir, _ = most_open_direction(ranges, 0.0, inc, min_r, max_r)
        self.scan_time = self.t

    def observe(self):
        if self.t >= self._next_scan:
            self._scan()
            self._next_scan += self.scan_period
        return Observation(t=self.t, x=self.x, y=self.y, yaw=self.yaw,
                           sectors=self.sectors, open_dir_deg=self.open_dir)

    def step(self, v, w):
        self.yaw += w * self.dt
        self.x += v * math.cos(self.yaw) * self.dt
        self.y += v * math.sin(self.yaw) * self.dt
        self.t += self.dt


def run(sim, plan, max_time=200.0, use_safety=True):
    """plan を実行し、統計を返す。"""
    executor = PlanExecutor()
    safety = SafetyFilter()
    events = list(executor.start(plan))
    min_clear = float("inf")
    path_len = 0.0
    safety_states = set()
    trace = []
    while sim.t < max_time:
        obs = sim.observe()
        cmd, ev = executor.tick(obs)
        events.extend(ev)
        v, w = cmd.v, cmd.w
        if use_safety:
            v, w, state = safety.filter(v, w, sim.sectors, sim.t - sim.scan_time)
            safety_states.add(state)
        px, py = sim.x, sim.y
        sim.step(v, w)
        path_len += math.hypot(sim.x - px, sim.y - py)
        min_clear = min(min_clear, sim.world.clearance(sim.x, sim.y))
        trace.append((sim.t, sim.x, sim.y, sim.yaw))
        if not executor.active:
            break
    return {
        "events": events,
        "types": [e["type"] for e in events],
        "min_clearance": min_clear,
        "path_len": path_len,
        "safety_states": safety_states,
        "trace": trace,
        "time": sim.t,
    }
