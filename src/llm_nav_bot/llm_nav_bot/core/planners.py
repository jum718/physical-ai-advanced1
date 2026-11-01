"""自然言語の指示を行動計画 (dict) に変換するプランナー。

- ClaudePlanner: Claude API (structured outputs) で計画を生成する。メインの AI 部分。
- RulePlanner:   キーワードによる簡易パーサ。API が使えない時のフォールバック兼、
                 レポートでの比較対象 (ベースライン)。

どちらも戻り値は「未検証の dict」で、plan_schema.validate_plan() に通してから使う。
"""

import json
import os
import re

from .plan_schema import PLAN_JSON_SCHEMA

REPLAN_PREFIX = "（自動再計画）"

SYSTEM_PROMPT = """\
あなたは Gazebo 上の TurtleBot3 Burger（差動二輪ロボット、360° 2D LiDAR 搭載）の行動計画 AI です。
ユーザーの自然言語の指示と、ロボットの現在のセンサー状態を読み、実行する行動ステップの列を JSON で出力してください。

## 使える行動 (action)
- forward: 前進。distance_m [m]、speed [m/s, 0.05〜0.18]
- backward: 後退。distance_m、speed
- turn: その場で回転。angle_deg（正 = 左 / 反時計回り、負 = 右 / 時計回り）
- turn_to_open: LiDAR で最も開けた方向を向く
- wall_follow: 壁に沿って走る。side（left / right）、target_distance_m（壁との距離 0.25〜1.0）、duration_s、speed
- explore: 障害物を避けながら自由に動き回る。duration_s、speed
- stop: 停止して待つ。duration_s
各ステップでその行動が使わないフィールドは null にしてください。

## 計画のルール
- 低レベルの安全層が別にあり、障害物に近づきすぎると強制停止します。あなたは衝突しにくい計画を立てることに集中してください。
- ユーザーが指示した行動の内容と順序は変えないこと。行動を削除したり、並べ替えたりしてはいけません。安全のために必要な準備の行動（回転など）は、該当のステップの直前に「追加」するだけにしてください。追加した場合は understanding に理由を短く書き、「（追加）」と明示する。
- 「現在のロボット状態」の各方向の距離 [m] を必ず考慮してください。例: 前方が 0.5m 未満で前進を指示された場合は、前進の直前に turn_to_open を追加する（指示された前進は残す）。
- 速度の指定がなければ 0.12 m/s 程度。「ゆっくり」は 0.06、「速く」は 0.18。
- 「少し」は 0.3m または 30° 程度。指示が曖昧なら妥当な値を選び、その解釈を understanding に短く書く。
- 「直近のやりとり」がある場合は文脈として使ってよい（「さっきの逆」「もう一回」など）。
- 実行できない指示（飛ぶ、物を掴む、地図上の固有の場所へ行く等）は steps を空にし、reply で理由と代わりにできることを伝える。
- repeat は計画全体の繰り返し回数（通常 1、最大 5）。
- understanding: 判断の要点だけを日本語 1 文・50 字以内で書く。例:「前方0.4mと近いので、先に開けた方を向く」「指示どおり0.5m前進」。距離の列挙や、行動の言い換え（計画に書いてあること）はしない。追加した行動がある場合は、その理由を必ず含める（「（追加）」と付ける）。
- reply: ユーザーへの返答を日本語 1 文・30 字以内で書く。例:「0.5m進みます」「空は飛べませんが、走る・回る・探索ならできます」
- どちらも、謝罪・前置き・長い説明は不要。
- 指示が「（自動再計画）」で始まる場合は、直前の計画が障害物で中断されています。センサー状態を見て、安全に元の目的へ近づく回復計画を立ててください。
"""


class PlannerError(Exception):
    pass


def format_user_message(instruction, status, history):
    """Claude に渡すユーザーメッセージを組み立てる。

    会話履歴は assistant ターンとして再送せず、テキストで要約して 1 つの user メッセージに入れる
    (毎回独立した 1 リクエストにすることで、履歴の編集・切り詰めが自由にできる)。
    """
    lines = ["## 現在のロボット状態"]
    if status:
        sectors = status.get("sectors", {})
        names = [("front", "前"), ("front_left", "左前"), ("left", "左"),
                 ("back", "後ろ"), ("right", "右"), ("front_right", "右前")]
        dist = "、".join(f"{label} {sectors[k]:.2f}m" for k, label in names if k in sectors)
        lines.append(f"- 障害物までの距離: {dist}")
        if "open_dir_deg" in status:
            lines.append(f"- 最も開けた方向: {status['open_dir_deg']:+.0f}°（正 = 左）")
        pose = status.get("pose")
        if pose:
            lines.append(f"- odom 位置: x={pose['x']:.2f}m, y={pose['y']:.2f}m, "
                         f"向き {pose['yaw_deg']:+.0f}°")
        if status.get("active"):
            lines.append(f"- 実行中の行動: {status.get('action')}")
    else:
        lines.append("- (センサー情報なし)")

    if history:
        lines.append("")
        lines.append("## 直近のやりとり（古い順）")
        for h in history:
            lines.append(f"- 指示「{h['instruction']}」→ 計画: {h['summary']} → 結果: {h['result']}")

    lines.append("")
    lines.append("## 新しい指示")
    lines.append(instruction)
    return "\n".join(lines)


class ClaudePlanner:
    name = "claude"

    # server-side refusal fallback ("fallbacks": "default") に対応するモデル
    _FALLBACK_MODELS = ("claude-opus-5-5", "claude-opus-5", "claude-fable-5-1",
                        "claude-sonnet-5-5")

    def __init__(self, model="claude-opus-5-5", effort="low", timeout_s=30.0,
                 use_fallback=True):
        try:
            import anthropic
        except ImportError as e:  # pragma: no cover - 環境依存
            raise PlannerError(
                "anthropic パッケージがありません: pip install anthropic") from e
        self._anthropic = anthropic
        # API キーは環境変数 ANTHROPIC_API_KEY から読む (コードに書かない)。
        # コピー時に混ざりがちな前後の空白・改行・引用符は取り除く
        # (末尾に改行があると HTTP ヘッダ不正になり、接続エラーに見える)。
        key = os.environ.get("ANTHROPIC_API_KEY", "").strip().strip("\"'").strip()
        if key and not key.isascii():
            raise PlannerError("ANTHROPIC_API_KEY に ASCII 以外の文字が含まれています "
                               "(説明文のプレースホルダをそのまま入れていませんか?)")
        # キー未設定のときは SDK の既定の認証 (ant auth login のプロファイル等) に任せる
        # Accept-Encoding: identity で圧縮なしの応答を要求する。コンテナ内の brotli/zstd
        # ライブラリと SDK の HTTP クライアントの版が合わないと、成功応答の展開時に
        # "TypeError: process() takes no keyword arguments" になり接続エラーに見えるため。
        # 応答は数百バイトの JSON なので、圧縮しなくても影響はない。
        self.client = anthropic.Anthropic(api_key=key or None, timeout=timeout_s,
                                          max_retries=1,
                                          default_headers={"Accept-Encoding": "identity"})
        self.model = model
        self.effort = effort
        self.use_fallback = use_fallback and model in self._FALLBACK_MODELS

    def plan(self, instruction, status=None, history=None):
        kwargs = dict(
            model=self.model,
            max_tokens=16000,
            system=SYSTEM_PROMPT,
            messages=[{"role": "user",
                       "content": format_user_message(instruction, status, history)}],
            output_config={
                "effort": self.effort,
                "format": {"type": "json_schema", "schema": PLAN_JSON_SCHEMA},
            },
        )
        a = self._anthropic
        try:
            if self.use_fallback:
                response = self.client.beta.messages.create(
                    **kwargs, betas=["server-side-fallback-2026-07-01"], fallbacks="default")
            else:
                response = self.client.messages.create(**kwargs)
        except a.AuthenticationError as e:
            raise PlannerError("API キーが無効です (ANTHROPIC_API_KEY を確認)") from e
        except a.RateLimitError as e:
            raise PlannerError("レート制限に達しました") from e
        except a.APIStatusError as e:
            raise PlannerError(f"API エラー {e.status_code}: {e.message}") from e
        except a.APIConnectionError as e:
            causes, x = [], e.__cause__
            while x is not None:
                causes.append(f"{type(x).__name__}: {str(x)[:80]}")
                x = x.__cause__
            raise PlannerError("API に接続できません (ネットワーク、またはキーの文字を確認)。"
                               f"原因: {' <- '.join(causes) or e}") from e

        if response.stop_reason == "refusal":
            raise PlannerError("モデルが応答を拒否しました")
        if response.stop_reason == "max_tokens":
            raise PlannerError("出力が max_tokens で途切れました")
        text = next((b.text for b in response.content if b.type == "text"), None)
        if not text:
            raise PlannerError("応答にテキストがありません")
        try:
            return json.loads(text)
        except json.JSONDecodeError as e:
            raise PlannerError(f"JSON を解釈できません: {text[:80]}") from e


# ----------------------------------------------------------------------
# ルールベースのフォールバック
# ----------------------------------------------------------------------
_CLAUSE_SPLIT = re.compile(r"[、。,，\n]|それから|そのあと|その後|次に|[てで]から|したら")
_DIST = re.compile(r"(\d+(?:\.\d+)?)\s*(m|メートル|cm|センチ)")
_DEG = re.compile(r"(\d+(?:\.\d+)?)\s*(度|°)")
_SEC = re.compile(r"(\d+(?:\.\d+)?)\s*(秒|s\b)")
_REPEAT = re.compile(r"(\d+)\s*回(繰り返|くりかえ)")


def _distance(clause, default):
    m = _DIST.search(clause)
    if not m:
        return default
    value = float(m.group(1))
    return value / 100.0 if m.group(2) in ("cm", "センチ") else value


def _seconds(clause, default):
    m = _SEC.search(clause)
    return float(m.group(1)) if m else default


def _speed(clause):
    if "ゆっくり" in clause:
        return 0.06
    if "速く" in clause or "急いで" in clause:
        return 0.18
    return 0.12


def _step(action, **kw):
    s = {"action": action, "distance_m": None, "angle_deg": None, "duration_s": None,
         "speed": None, "side": None, "target_distance_m": None}
    s.update(kw)
    return s


def _parse_clause(clause):
    c = clause.strip()
    if not c:
        return None
    if any(k in c for k in ("止ま", "停止", "ストップ", "待")):
        return _step("stop", duration_s=_seconds(c, 1.0))
    if "壁" in c:
        return _step("wall_follow", side="left" if "左" in c else "right",
                     target_distance_m=_distance(c, 0.4),
                     duration_s=_seconds(c, 20.0), speed=_speed(c))
    if any(k in c for k in ("探索", "自由", "うろうろ", "歩き回", "動き回", "パトロール")):
        return _step("explore", duration_s=_seconds(c, 30.0), speed=_speed(c))
    if any(k in c for k in ("広い", "開けた", "空いて")):
        return _step("turn_to_open")
    if any(k in c for k in ("Uターン", "後ろを向", "反対を向", "振り返")):
        return _step("turn", angle_deg=180.0)
    if _DEG.search(c) or any(k in c for k in ("回転", "回って", "向い", "向き", "曲が",
                                               "ターン", "回れ")):
        if "一回転" in c or "1回転" in c:
            angle = 360.0
        else:
            m = _DEG.search(c)
            angle = float(m.group(1)) if m else (30.0 if "少し" in c else 90.0)
        return _step("turn", angle_deg=-angle if "右" in c else angle)
    if any(k in c for k in ("後ろ", "後退", "下が", "バック", "戻")):
        return _step("backward", distance_m=_distance(c, 0.3), speed=_speed(c))
    if any(k in c for k in ("前", "進", "まっすぐ", "直進")):
        return _step("forward", distance_m=_distance(c, 0.3 if "少し" in c else 0.5),
                     speed=_speed(c))
    return None


class RulePlanner:
    name = "rule"

    def plan(self, instruction, status=None, history=None):
        if instruction.startswith(REPLAN_PREFIX):
            return {
                "understanding": "ルールベース: 障害物で中断されたため、少し下がって開けた方向を向く。",
                "reply": "障害物を避けるため、少し下がって向きを変えます。",
                "steps": [_step("backward", distance_m=0.2, speed=0.08),
                          _step("turn_to_open")],
                "repeat": 1,
            }
        steps, unknown = [], []
        for clause in _CLAUSE_SPLIT.split(instruction):
            if clause is None or not clause.strip():
                continue
            s = _parse_clause(clause)
            if s is None:
                unknown.append(clause.strip())
            else:
                steps.append(s)
        m = _REPEAT.search(instruction)
        understanding = "ルールベース解析: " + (
            " → ".join(s["action"] for s in steps) if steps else "該当する行動なし")
        if unknown:
            understanding += f"（解釈できなかった部分: {'／'.join(unknown)}）"
        return {
            "understanding": understanding,
            "reply": "了解しました。" if steps else "指示を理解できませんでした。",
            "steps": steps,
            "repeat": int(m.group(1)) if m else 1,
        }


def is_stop_command(text):
    """LLM を通さず即座に止めるべき指示か (遅延なしの緊急停止)。"""
    t = text.strip().lower()
    if len(t) > 12:
        return False
    return any(k in t for k in ("止ま", "とまれ", "止めて", "ストップ", "停止", "stop", "やめ"))
