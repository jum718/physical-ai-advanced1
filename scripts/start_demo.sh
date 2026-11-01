#!/usr/bin/env bash
# デモを初期状態から起動する (WSL / Linux のシェルで実行)。
#
#   bash scripts/start_demo.sh            # Claude で動かす (API キーを聞かれる)
#   bash scripts/start_demo.sh rule       # API なし (ルールベース)
#   GUI=false bash scripts/start_demo.sh  # Gazebo を画面なしで起動
#   NO_CONSOLE=1 bash scripts/start_demo.sh  # コンソールを開かず、起動だけ行う
#
# 古いプロセスを止めて、シミュレーション (ロボットは初期位置に戻る) → 各ノード → コンソール
# の順に起動する。API キーは環境変数 ANTHROPIC_API_KEY があればそれを使い、なければ
# 画面に表示せずに入力させる (シェルの履歴にも、録画にも残らない)。
set -euo pipefail

CONTAINER="${CONTAINER:-physical-ai-turtlebot3-gui}"
BACKEND="${1:-claude}"
GUI="${GUI:-true}"

if [ "$BACKEND" = "claude" ]; then
  if [ -z "${ANTHROPIC_API_KEY:-}" ]; then
    read -r -s -p "ANTHROPIC_API_KEY (入力は表示されません): " ANTHROPIC_API_KEY
    echo
  fi
  # コピー時に混ざる空白・改行を除く
  ANTHROPIC_API_KEY="$(printf %s "$ANTHROPIC_API_KEY" | tr -d '[:space:]"'"'")"
  case "$ANTHROPIC_API_KEY" in
    "") echo "キーが空です" >&2; exit 1 ;;
    *[!A-Za-z0-9_-]*) echo "キーに使えない文字が含まれています (説明文をそのまま入力していませんか?)" >&2; exit 1 ;;
  esac
  export ANTHROPIC_API_KEY
else
  export ANTHROPIC_API_KEY=""
fi

ENV_SETUP='source /opt/ros/jazzy/setup.bash && source /root/ws/install/setup.bash && export TURTLEBOT3_MODEL=burger'

docker start "$CONTAINER" >/dev/null

if [ "$BACKEND" = "claude" ]; then
  # 無効なキーのまま起動すると、ルールベースへ自動で切り替わって気づきにくいので先に検査する
  # Accept-Encoding は planners.py と同じ設定 (圧縮された応答の展開エラーを避ける)
  if ! docker exec -e ANTHROPIC_API_KEY "$CONTAINER" python3 -c "
import sys, anthropic
try:
    anthropic.Anthropic(max_retries=1, timeout=15, default_headers={'Accept-Encoding': 'identity'}).models.retrieve('claude-opus-5-5')
except Exception as e:
    print('原因:', type(e).__name__, getattr(e, 'status_code', ''), repr(e.__cause__ or getattr(e, 'message', ''))[:200], file=sys.stderr)
    sys.exit(1)
"; then
    echo "API キーを確認できません。上の原因を確認してください。" >&2
    exit 1
  fi
  echo "API キーを確認しました"
fi

echo "[1/4] 古いプロセスを停止"
# 正規表現の [x] 形式は、このコマンド自身にマッチしないための工夫
docker exec "$CONTAINER" bash -c "
  pkill -f '[l]ib/llm_nav_bot/'; pkill -f '[r]os2 launch'; pkill -f '[g]z sim'
  pkill -f '[r]uby'; pkill -f '[p]arameter_bridge'; pkill -f '[r]obot_state_publisher'
  sleep 3; pkill -9 -f '[g]z sim'; pkill -9 -f '[r]uby'; true"

echo "[2/4] Gazebo を起動 (GUI=$GUI)"
docker exec -d "$CONTAINER" bash -lc "$ENV_SETUP && ros2 launch llm_nav_bot sim.launch.py gui:=$GUI > /tmp/sim.log 2>&1"
for _ in $(seq 1 60); do
  if docker exec "$CONTAINER" bash -lc "source /opt/ros/jazzy/setup.bash && ros2 topic list 2>/dev/null | grep -q '^/scan\$'"; then
    break
  fi
  sleep 2
done
sleep 5  # ロボットの生成とブリッジの接続を待つ

echo "[3/4] 行動層・安全層・LLM 指令ノードを起動 (backend=$BACKEND)"
docker exec -d -e ANTHROPIC_API_KEY "$CONTAINER" bash -lc \
  "$ENV_SETUP && ros2 launch llm_nav_bot llm_nav_bot.launch.py backend:=$BACKEND > /tmp/nav.log 2>&1"
sleep 6
docker exec "$CONTAINER" bash -lc "grep -m1 -E 'プランナー|初期化できません' /tmp/nav.log" || true

if [ "${NO_CONSOLE:-}" = "1" ]; then
  echo "[4/4] コンソールは開きません (NO_CONSOLE=1)"
  exit 0
fi
echo "[4/4] コンソールを開きます"
docker exec -it "$CONTAINER" bash -lc "$ENV_SETUP && ros2 run llm_nav_bot console"
