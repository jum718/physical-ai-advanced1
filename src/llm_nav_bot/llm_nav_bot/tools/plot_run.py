"""data_logger の出力をグラフと要約にする (ROS 不要。Windows でも実行可)。

使い方:
  python3 -m llm_nav_bot.tools.plot_run [run_dir]      # 省略時は ~/llm_nav_logs の最新
  ros2 run llm_nav_bot plot_run [run_dir]

出力 (run_dir 内): trajectory.png, distances.png, summary.txt
"""

import csv
import glob
import json
import math
import os
import sys

STOP_STATES = ("STOP_FRONT", "STOP_BACK", "NO_SCAN")


def _float(v, default=math.nan):
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def load(run_dir):
    with open(os.path.join(run_dir, "trajectory.csv"), encoding="utf-8") as f:
        traj = list(csv.DictReader(f))
    events = []
    path = os.path.join(run_dir, "events.csv")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            events = list(csv.DictReader(f))
    return traj, events


def summarize(traj, events):
    xs = [_float(r["x"]) for r in traj]
    ys = [_float(r["y"]) for r in traj]
    dist = sum(math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) for i in range(1, len(xs)))
    clear = [_float(r["forward_clearance"]) for r in traj]
    clear = [c for c in clear if math.isfinite(c)]
    sides = []
    for r in traj:
        vals = [_float(r[k]) for k in ("front", "front_left", "left", "front_right", "right",
                                       "back")]
        vals = [v for v in vals if math.isfinite(v)]
        if vals:
            sides.append(min(vals))
    llm = [json.loads(e["detail"]) for e in events if e["source"] == "llm"]
    latencies = [r["latency_s"] for r in llm if not r["backend"].startswith("fast")]
    plan_types = [e["type"] for e in events if e["source"] == "plan"]
    t0 = _float(traj[0]["t"]) if traj else 0.0
    t1 = _float(traj[-1]["t"]) if traj else 0.0
    return {
        "duration_s": round(t1 - t0, 1),
        "distance_m": round(dist, 2),
        "commands": sum(1 for e in events if e["source"] == "user"),
        "plans": len(llm),
        "replans": sum(1 for r in llm if r["instruction"].startswith("（自動再計画）")),
        "plans_finished": plan_types.count("plan_finished"),
        "plans_blocked": plan_types.count("plan_blocked"),
        "llm_latency_mean_s": round(sum(latencies) / len(latencies), 2) if latencies else None,
        "llm_latency_max_s": round(max(latencies), 2) if latencies else None,
        "fallback_to_rule": sum(1 for r in llm if r["backend"] == "rule(fallback)"),
        "output_corrections": sum(len(r.get("warnings") or []) for r in llm),
        "safety_interventions": sum(1 for e in events
                                    if e["source"] == "safety"
                                    and e["type"] in ("STOP_FRONT", "STOP_BACK")),
        "min_forward_clearance_m": round(min(clear), 3) if clear else None,
        "min_obstacle_distance_m": round(min(sides), 3) if sides else None,
    }


def plot(run_dir, traj, events):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    xs = [_float(r["x"]) for r in traj]
    ys = [_float(r["y"]) for r in traj]
    ts = [_float(r["t"]) for r in traj]
    actions = [r["action"] or "idle" for r in traj]

    # 1) 軌跡 (行動ごとに色分け、安全層の停止位置に×印)
    fig, ax = plt.subplots(figsize=(7, 7))
    names = sorted(set(actions))
    cmap = plt.get_cmap("tab10")
    for i, name in enumerate(names):
        px = [x for x, a in zip(xs, actions) if a == name]
        py = [y for y, a in zip(ys, actions) if a == name]
        ax.scatter(px, py, s=6, color=cmap(i % 10), label=name)
    stops = [(x, y) for x, y, r in zip(xs, ys, traj) if r["safety_state"] in STOP_STATES]
    if stops:
        ax.scatter(*zip(*stops), marker="x", s=60, color="red", label="safety stop")
    if xs:
        ax.plot(xs[0], ys[0], "k^", ms=10, label="start")
        ax.plot(xs[-1], ys[-1], "ks", ms=8, label="end")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m] (odom)")
    ax.set_ylabel("y [m] (odom)")
    ax.set_title("Trajectory by behavior")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(run_dir, "trajectory.png"), dpi=150)
    plt.close(fig)

    # 2) 前方の空き距離と速度の時系列 (指示のタイミングを縦線で表示)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    a1.plot(ts, [_float(r["forward_clearance"]) for r in traj], label="forward clearance")
    a1.plot(ts, [_float(r["right"]) for r in traj], alpha=0.6, label="right")
    a1.plot(ts, [_float(r["left"]) for r in traj], alpha=0.6, label="left")
    a1.axhline(0.22, color="red", ls="--", lw=1, label="safety stop (0.22m)")
    a1.set_ylabel("distance [m]")
    a1.set_ylim(0, 2.0)
    a1.legend(fontsize=8, loc="upper right")
    a1.grid(alpha=0.3)
    a2.plot(ts, [_float(r["v"]) for r in traj], label="v [m/s]")
    a2.plot(ts, [_float(r["w"]) for r in traj], label="w [rad/s]", alpha=0.7)
    a2.set_xlabel("sim time [s]")
    a2.legend(fontsize=8, loc="upper right")
    a2.grid(alpha=0.3)
    for e in events:
        if e["source"] == "llm":
            for a in (a1, a2):
                a.axvline(_float(e["t"]), color="gray", lw=0.8, alpha=0.6)
    a1.set_title("Distances / velocities (gray lines = new plan)")
    fig.tight_layout()
    fig.savefig(os.path.join(run_dir, "distances.png"), dpi=150)
    plt.close(fig)


def latest_run(base=os.path.expanduser("~/llm_nav_logs")):
    runs = sorted(glob.glob(os.path.join(base, "*", "trajectory.csv")))
    return os.path.dirname(runs[-1]) if runs else None


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    run_dir = argv[0] if argv else latest_run()
    if not run_dir:
        print("ログが見つかりません。run_dir を指定してください。")
        return 1
    traj, events = load(run_dir)
    if not traj:
        print("trajectory.csv が空です。")
        return 1
    stats = summarize(traj, events)
    text = "\n".join(f"{k}: {v}" for k, v in stats.items())
    with open(os.path.join(run_dir, "summary.txt"), "w", encoding="utf-8") as f:
        f.write(text + "\n")
    print(f"[{run_dir}]\n{text}")
    try:
        plot(run_dir, traj, events)
        print("→ trajectory.png / distances.png を保存しました")
    except ImportError:
        print("matplotlib がないためグラフは省略しました (apt install python3-matplotlib)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
