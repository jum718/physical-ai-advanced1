"""LiDAR に基づく安全層 (LLM・行動層の指令より優先される最終フィルタ)。

行動層とは独立に、速度指令と最新のスキャンだけを見て判断する。
LLM が誤った計画を出しても、ここで衝突を防ぐのが設計上の要点。
"""

from dataclasses import dataclass

from .scan_utils import forward_clearance

OK = "OK"
SLOW = "SLOW"
STOP_FRONT = "STOP_FRONT"
STOP_BACK = "STOP_BACK"
NO_SCAN = "NO_SCAN"


@dataclass
class SafetyParams:
    stop_distance: float = 0.22       # 前方がこれ未満なら前進成分を 0 にする [m]
    slow_distance: float = 0.45       # これ未満では距離に応じて前進速度を減速 [m]
    back_stop_distance: float = 0.20  # 後方がこれ未満なら後退成分を 0 にする [m]
    scan_timeout: float = 1.0         # スキャンがこれ以上古ければ全停止 [s, シミュ時間]
    max_linear: float = 0.20
    max_angular: float = 1.2


class SafetyFilter:
    def __init__(self, params=None):
        self.p = params or SafetyParams()

    def filter(self, v, w, sectors, scan_age):
        """(v, w) を安全な値に補正して (v, w, state) を返す。

        その場回転は常に許可する (前が塞がっても向きを変えて脱出できるように)。
        """
        p = self.p
        if sectors is None or scan_age is None or scan_age > p.scan_timeout:
            return 0.0, 0.0, NO_SCAN

        v = max(-p.max_linear, min(p.max_linear, v))
        w = max(-p.max_angular, min(p.max_angular, w))
        state = OK

        if v > 0.0:
            clear = forward_clearance(sectors)
            if clear < p.stop_distance:
                v, state = 0.0, STOP_FRONT
            elif clear < p.slow_distance:
                scale = (clear - p.stop_distance) / (p.slow_distance - p.stop_distance)
                v *= max(0.2, scale)
                state = SLOW
        elif v < 0.0 and sectors["back"] < p.back_stop_distance:
            v, state = 0.0, STOP_BACK
        return v, w, state
