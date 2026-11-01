"""LiDAR (LaserScan) を扇形セクターごとの最小距離に要約するユーティリティ。

ROS に依存しない純粋な関数だけを置き、Windows 上でも単体テストできるようにしている。
角度は「ロボット正面 = 0°、左 = +、右 = -」(ROS の REP-103 と同じ向き)。
"""

import math

# セクター名: (開始角, 終了角) [deg]。開始 > 終了 のときは ±180° をまたぐ扇形。
DEFAULT_SECTORS = {
    "front": (-20.0, 20.0),
    "front_left": (20.0, 60.0),
    "left": (60.0, 120.0),
    "back": (150.0, -150.0),
    "right": (-120.0, -60.0),
    "front_right": (-60.0, -20.0),
    "front_wide": (-45.0, 45.0),
}


def normalize_deg(angle_deg):
    """角度を (-180, 180] に正規化する。"""
    a = math.fmod(angle_deg + 180.0, 360.0)
    if a <= 0.0:
        a += 360.0
    return a - 180.0


def normalize_rad(angle_rad):
    """角度を (-pi, pi] に正規化する。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _in_sector(angle_deg, lo, hi):
    if lo <= hi:
        return lo <= angle_deg <= hi
    return angle_deg >= lo or angle_deg <= hi


def _is_valid(r, range_min, range_max):
    return math.isfinite(r) and range_min <= r <= range_max


CORRIDOR_HALF_WIDTH = 0.18   # ロボット半径 (約 0.105m) + 余裕 [m]
CORRIDOR_MIN_AHEAD = 0.08    # これより手前 (真横付近) の点は「進路上」に数えない [m]


def sector_distances(ranges, angle_min, angle_increment, range_min, range_max,
                     sectors=DEFAULT_SECTORS, corridor_half_width=CORRIDOR_HALF_WIDTH):
    """各セクター内の最小距離 [m] を返す。

    inf / nan / range_min 未満の値は無視する。
    有効な点が 1 つもないセクターは「障害物なし」とみなし range_max を返す。

    "corridor" は進路の幅 (左右 corridor_half_width) の中にある点までの
    前方距離 (x 成分) の最小値。壁のすぐ横を通るだけでは小さくならない。
    """
    result = {name: float(range_max) for name in sectors}
    corridor = float(range_max)
    for i, r in enumerate(ranges):
        if not _is_valid(r, range_min, range_max):
            continue
        rad = angle_min + i * angle_increment
        x, y = r * math.cos(rad), r * math.sin(rad)
        if x >= CORRIDOR_MIN_AHEAD and abs(y) <= corridor_half_width and x < corridor:
            corridor = x
        a = normalize_deg(math.degrees(rad))
        for name, (lo, hi) in sectors.items():
            if r < result[name] and _in_sector(a, lo, hi):
                result[name] = float(r)
    result["corridor"] = corridor
    return result


def forward_clearance(sectors):
    """前進方向の実質的な空き距離 [m]。

    進路の幅の中にある障害物までの距離 ("corridor") と、真正面 (±20°) の最小距離の
    小さい方。以前は斜め前 (±45°) の扇形を使っていたが、壁のすぐ横を走るだけで
    「前が塞がっている」と判定してしまったため、ロボットが通る幅で判定する。
    "corridor" がない辞書 (古い形式) では従来の式にフォールバックする。
    """
    if "corridor" in sectors:
        return min(sectors["front"], sectors["corridor"])
    return min(sectors["front"], sectors["front_wide"] * 1.3)


def most_open_direction(ranges, angle_min, angle_increment, range_min, range_max,
                        window_deg=30, step_deg=10):
    """最も開けている方向 [deg] と、その窓内の平均距離 [m] を返す。

    1° ごとのビンに最小距離を詰め、window_deg 幅の移動平均が最大になる中心角を選ぶ。
    同点の場合は正面に近い方向を優先する。
    """
    bins = [None] * 360
    for i, r in enumerate(ranges):
        if math.isnan(r) or r < range_min:
            continue
        r = min(r, range_max)  # inf (計測範囲外) は range_max 扱い
        a = normalize_deg(math.degrees(angle_min + i * angle_increment))
        b = int(round(a)) % 360
        if bins[b] is None or r < bins[b]:
            bins[b] = r
    filled = [range_max if v is None else v for v in bins]

    half = window_deg // 2
    best_angle, best_mean = 0.0, -1.0
    for center in range(-180, 180, step_deg):
        total = 0.0
        for d in range(-half, half + 1):
            total += filled[(center + d) % 360]
        mean = total / (2 * half + 1)
        better = mean > best_mean + 1e-6
        tie_closer = abs(mean - best_mean) <= 1e-6 and abs(center) < abs(best_angle)
        if better or tie_closer:
            best_angle, best_mean = float(center), mean
    return best_angle, best_mean
