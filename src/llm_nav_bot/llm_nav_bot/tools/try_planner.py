"""Claude プランナー単体の動作確認 (ROS・Gazebo 不要)。

  export ANTHROPIC_API_KEY=...        # キーはコードやチャットに書かない
  python3 -m llm_nav_bot.tools.try_planner              # 組み込みの例文を一通り試す
  python3 -m llm_nav_bot.tools.try_planner "右の壁に沿って30秒走って"

センサー状態は固定のダミー (前方 0.4m・右が壁) を渡す。
出力: LLM の生の計画 → 検証後の計画 → 補正された項目、応答時間、
      同じ指示に対するルールベースの結果 (比較用)。
"""

import sys
import time

from llm_nav_bot.core.plan_schema import validate_plan
from llm_nav_bot.core.planners import ClaudePlanner, PlannerError, RulePlanner

DUMMY_STATUS = {
    "sectors": {"front": 0.40, "front_left": 1.2, "left": 1.5, "back": 2.0,
                "right": 0.55, "front_right": 0.8},
    "open_dir_deg": 60.0,
    "pose": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0},
    "active": False,
}

EXAMPLES = [
    "1m進んでから左に90度回って",
    "右の壁に沿ってゆっくり30秒走って",
    "ぶつからないように部屋を1分探索して",
    "一番広い方向を向いて、50cm進んで",
    "0.5mの正方形を描いて",
    "なんとなく前に進んで",          # 曖昧な指示 (ルールベースが苦手な例)
    "空を飛んで",                    # 実行不能な指示
    "10m全速力で前に進んで",         # 上限を超える指示 (補正されるはず)
]


def show(title, raw):
    plan, warnings = validate_plan(raw)
    print(f"  [{title}] {plan.summary()}")
    if plan.understanding:
        print(f"      解釈: {plan.understanding}")
    if plan.reply:
        print(f"      返答: {plan.reply}")
    for w in warnings:
        print(f"      補正: {w}")


def main(argv):
    instructions = argv or EXAMPLES
    try:
        claude = ClaudePlanner()
    except PlannerError as e:
        print(f"初期化に失敗: {e}")
        return 1
    rule = RulePlanner()
    failures = 0
    for text in instructions:
        print(f"\n指示: {text}")
        t0 = time.monotonic()
        try:
            raw = claude.plan(text, DUMMY_STATUS, [])
            print(f"  (Claude {claude.model}: {time.monotonic() - t0:.1f}s)")
            show("Claude", raw)
        except PlannerError as e:
            failures += 1
            print(f"  Claude 失敗: {e}")
        show("ルール", rule.plan(text, DUMMY_STATUS, []))
    print(f"\n失敗: {failures}/{len(instructions)}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
