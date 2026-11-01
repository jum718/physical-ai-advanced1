# llm_nav_bot — 自然言語で指示できる障害物回避ロボット

TurtleBot3 Burger (ROS 2 Jazzy + Gazebo) を、**日本語の指示**で動かす。
LLM (Claude) が指示を行動計画に変換し、反応型の制御と **LiDAR の安全層**が実行する。
Nav2・AMCL・地図・TF には依存しない。

```
「右の壁に沿って30秒走って」
        │  自然言語
        ▼
┌─────────────────┐  現在の LiDAR 状態 (前後左右の距離・開けた方向)
│  llm_commander   │◀───────────────────────────────┐
│  Claude で計画   │                                  │
└────────┬────────┘                                  │
         │ 行動計画 (JSON) ※出力は検証して上限内に補正│
         ▼                                            │
┌─────────────────┐  /scan, /odom                    │
│behavior_controller│─────────────────────────────────┤
│ 前進/旋回/壁沿い  │                                  │
│ /探索 を実行      │                                  │
└────────┬────────┘                                  │
         │ /cmd_vel_raw                               │
         ▼                                            │
┌─────────────────┐  /scan                           │
│  safety_layer    │ 前方が近いと並進を 0 に           │
│ (LLM より優先)   │ LiDAR 途絶・指令途絶で全停止      │
└────────┬────────┘                                  │
         │ /cmd_vel (Twist / TwistStamped を自動判別) │
         ▼                                            │
   Gazebo の TurtleBot3 ───────────── /scan ─────────┘
```

## 工夫した点

1. **LLM と安全の分離**: LLM は「何をするか」だけを決め、衝突回避は LLM の指示に関係なく
   LiDAR で動く安全層が担う。LLM が誤った計画を出しても、安全層は独立に止める。
2. **LLM の出力を信用しない**: 計画は JSON Schema (構造化出力) で受け取ったうえで、
   未知の行動の除去・速度/距離の上限クランプ・ステップ数の制限を行う (`plan_schema.py`)。
3. **センサー状態を踏まえた計画**: 前後左右の距離と最も開けた方向を LLM に渡し、
   「前方が近いので先に向きを変える」といった判断をさせる。曖昧な指示や実行不能な指示
   (「空を飛んで」) にも理由つきで答える。
4. **障害物で中断したら自動で再計画**: 計画が塞がれて中断されると、状況を添えて
   LLM に回復計画を作らせる (上限 2 回)。
5. **API が使えなくても止まらない**: 失敗時はルールベースのプランナーに自動で切り替える。
6. **シミュレーションが遅くても動く**: 制御は受信したスキャンと `/clock` 基準で動き、
   Gazebo の RTF が低くても挙動が変わらない。
7. **Nav2 を使わない理由**: 本プロジェクトの前半で Nav2 を試したが、低い RTF 環境で TF の
   時刻不整合・センサーメッセージの破棄・Collision Monitor の停止が起き、安定走行できなかった。
   そこで map→odom の TF に依存しない反応型の構成にした。

## 動作確認の状況

| 項目 | 状況 |
|---|---|
| コアロジックの単体テスト (2D シミュレータ上で衝突なしを検証) | 通過 |
| ROS 2 Jazzy でのビルド・5 ノードの起動 | 確認済み |
| Gazebo での前進・壁沿い 30 秒・探索 60 秒 | 確認済み (中断なし、安全層の介入 0 回、最小の障害物距離 0.27 m) |
| 安全層のみの試験 (行動層を介さず全速前進の指令) | 前方約 0.2 m で停止 |
| Claude による計画 (応答 3〜6 秒、構造化出力) | 確認済み |
| 障害物による中断 → 自動再計画 | 確認済み |
| 狭い場所での 0.5 m 四方の正方形 | 3 辺目で中断 (この環境では空間が足りない) |

## 動作環境

- Windows 11 + Docker Desktop (WSL2)
- ROS 2 Jazzy、Gazebo Sim 8 (Harmonic)、`turtlebot3_gazebo`、`ros_gz`
- Claude API のキー (`ANTHROPIC_API_KEY`)。キーなしでも `backend:=rule` で動く

## セットアップ

```bash
# コンテナ内で
source /opt/ros/jazzy/setup.bash
apt-get update && apt-get install -y python3-pip python3-matplotlib
pip install --break-system-packages anthropic
cd <このリポジトリ> && colcon build --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 実行

```bash
# 1) シミュレーション (画面なしでも起動できる。GUI は gui:=true)
ros2 launch llm_nav_bot sim.launch.py

# 2) 行動層・安全層・LLM 指令・ロガー
export ANTHROPIC_API_KEY=...        # キーは環境変数で渡す。コードやログに書かない
ros2 launch llm_nav_bot llm_nav_bot.launch.py                 # Claude
ros2 launch llm_nav_bot llm_nav_bot.launch.py backend:=rule   # API なし

# 3) 指示の入力 (別ターミナル)
ros2 run llm_nav_bot console
```

指示の例:

- `1m進んでから左に90度回って`
- `右の壁に沿って30秒走って`
- `ぶつからないように部屋を1分探索して`
- `0.5mの正方形を描いて`
- `止まって` (LLM を通さず即座に停止)

前の計画の実行中に新しい指示を出すと、前の計画は取り消されて置き換わる。

### Claude 単体の確認 (ROS・Gazebo 不要)

```bash
python3 -m llm_nav_bot.tools.try_planner                    # 例文を Claude とルールで比較
python3 -m llm_nav_bot.tools.try_planner "右の壁に沿って走って"
```

### 走行ログの評価

`data_logger` が `~/llm_nav_logs/<日時>/` に CSV を保存する。

```bash
ros2 run llm_nav_bot plot_run            # 最新のログを要約し、trajectory.png / distances.png を出力
```

要約には、走行距離、安全層の介入回数、中断・再計画の回数、LLM の応答時間、最小の障害物距離が出る。

## テスト (ROS 不要)

```powershell
# Windows
.\src\llm_nav_bot\run_tests.ps1
```

```bash
# Linux
cd src/llm_nav_bot && python3 -c "import sys; sys.path[:0]=['.','test']; import test_core as t; [getattr(t,n)() for n in dir(t) if n.startswith('test_')]"
```

LiDAR の扇形処理、計画の検証、各行動、安全層を、2D の簡易シミュレータ上で検証している。

## 構成

| ファイル | 役割 |
|---|---|
| `core/scan_utils.py` | LiDAR を前後左右の扇形に要約、最も開けた方向を探す |
| `core/plan_schema.py` | 行動計画のスキーマと検証・クランプ |
| `core/planners.py` | Claude プランナー、ルールベースのプランナー |
| `core/behaviors.py` | 前進・旋回・壁沿い・探索の実行 |
| `core/safety.py` | 安全層のロジック |
| `*_node.py` | 上記を ROS 2 ノードにしたもの |
| `launch/sim.launch.py` | ヘッドレス対応の Gazebo 起動 |
| `config/params.yaml` | 速度・距離などのパラメータ |

## 設定

`src/llm_nav_bot/config/params.yaml` で、速度の上限、安全層の停止距離 (既定 0.22 m) と減速距離
(0.45 m)、LLM のモデル・`effort`、再計画の回数などを変えられる。

## 既知の制約

- 自己位置は odom のみ (ドリフトあり)。長距離の正確な移動は対象外。
- 地図上の固有の場所 (「キッチンへ行って」) には対応していない。
- LiDAR は 2D。段差や、センサーの高さにない障害物は検知できない。
- LLM の応答に 3〜6 秒かかる。停止指示だけは LLM を通さず即時に処理する。
- 実機での動作は未検証。
