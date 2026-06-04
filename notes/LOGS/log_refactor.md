# Log: リポジトリリファクタ

## 2026-06-04: フラット構成 → モジュラーパッケージ + CLI 設定

### 動機
ルート直下に ~10 個の .py が散在し, `/workspace` 絶対パスが .py 内 27 箇所にハードコード,
設定は `run_stage1.py` の flat `Config` dataclass に埋め込まれ CLI 変更不可,
`results/` が 1.1GB に肥大化 (grid 動画ありは 3 run のみ), ルートに stale な
`verify_*.png` / `__pycache__` が散乱していた.

### 調査 (web)
PyPA の src-vs-flat 議論, tyro, robosuite / CleanRL / mujoco_menagerie の構成を確認:
- 1-dev・未 publish のリポジトリには src/ レイアウトは過剰 → ルート直下に単一 import パッケージ
- CLI 設定は tyro (型注釈 dataclass から CLI 自動生成, ネスト・override 対応)
- scene/asset 分離 (menagerie 流), 拡張は Protocol + 小レジストリのみ (entry_points は YAGNI)

### 実施
- `pusher_slider/` パッケージ新設 + `pyproject.toml` で editable install (`pixi.toml` に tyro 追加)
- モジュール分割:
  - `paths` — リポジトリ相対の scene/asset/results 解決 (`/workspace` ハードコード除去の要)
  - `config` — tyro 用ネスト dataclass (`SimConfig`: `push`/`mpc`/`robot`/`render`)
  - `kinematics` — 共有 IK ヘルパ (旧 `run_stage1` から抽出, keyframe との循環 import を解消)
  - `io` — trial ディレクトリ / `config.json` dump / `Log`+npz
  - `controllers` — `Controller` Protocol + dict レジストリ (`make_controller`), `mpc.py` は本体不変
  - `sim/runner` (run ループ), `sim/keyframe` (6-DOF IK)
  - `analytical/push_com_sim` (Stage 0, 出力先ハードコードバグ `/mnt/user-data/...` を修正)
  - `viz/grid_video`, `viz/plots`
- XML を `scenes/` へ移設 (旧 pusher を `scenes/legacy/`), `meshdir` を相対修正 (`../assets`)
- `scripts/` に tyro CLI エントリ 4 本, `pixi run push/render/keyframe/analytical` タスク追加
- experiments 6 本を新 API へ移行 (flat→nested config アクセス, paths 経由). 物理は各自の scene 温存
- results 整理: grid 動画の無い run + `exp_*/` を削除 (1.1GB→87MB), `verify_*.png` を git rm

### 検証
- `pixi run python scripts/run_push.py` (デフォルト): ゴール到達, x=−3.6mm, θ=−0.01° (リファクタ前と同等)
- keyframe 生成器がデプロイ済み qpos を完全再現
- grid 動画生成 OK, `pytest` pass (scene ロード+安定性), `ruff check/format` clean (pusher_slider/scripts/tests)

### 既知の制約 (リファクタ起因ではない)
- `exp_contact_sensitivity` / `exp_pressure_distribution` は scene テキストの
  `<include file="ur5e_with_pusher.xml"/>` を文字列置換する設計だが, gripper 化以降
  stage1 scene は gripper を include しており置換対象が存在しない (gripper 化時点からの既存問題).
  パス移行のみ実施し, 物理・ロジックは温存. これらを動かすには別途 scene 注入ロジックの更新が必要.

## 2026-06-04: 論文対応の注釈 + motion cone 式バグ修正

Hogan WAFR2016 (main.pdf) 本文を pymupdf でテキスト/画像抽出し, MPC 実装
(`controllers/mpc.py`) の各部に論文の節・式番号の対応を注釈として記述した
(§4.2-4.3 限界曲面→`_limit_surface_c`, Eq.(2)(3)→`_motion_cone`, Eq.(4)-(7)→`_select_mode`,
Eq.(8)→`_compute_B`/`compute_control`, §5 Eq.(16)→`_solve_single_schedule`, §5.2 FOM→`_solve_fom_qp`).

照合の過程で `_motion_cone` の式が Eq.(2)(3) と不一致 (px²/py² 入替・cross 項符号反転) と判明.
限界曲面から独立に再導出して論文式が正しいことを確認し, 実装を修正:
- 修正前: `γt=(µc²−pxpy+µpy²)/(c²+px²+µpxpy)`
- 修正後: `γt=(µc²−pxpy+µpx²)/(c²+py²−µpxpy)` (γb も対応して修正)

検証: (1) 修正版は任意接触で cone が 0 を跨ぐ (バグ版は off-center で跨がない). (2) フル sim
(中心化運転) は回帰なし (x=−3.0mm, θ=−0.01°, ゴール到達). (3) off-center collapse は許容偏心が
~2 倍に拡大 (px≈0.004→≈0.01) したが大偏心+回転では残存 → cone バグは collapse の寄与因子で
あり単独原因ではなかった (詳細は ISSUES). 当初「Issue #2 の真因」と述べたのは誤りで, 訂正した.

PLAN/TODO の MPC 忠実度メモも併せて更新. 残課題は ISSUES の off-center collapse のみ.
