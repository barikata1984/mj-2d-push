# TODO

## Stage 0: 解析 LS シミュレータ

- [x] pusher-slider 解析モデル実装 (`push_com_sim.py`)
- [x] CoM 推定 + 条件 A/B/C 対照実験で Δθ 検証
- [x] 結果: A ≫ B ≈ C の対照成立を確認

## Stage 1: MuJoCo シミュレーション

- [x] MuJoCo + UR5e menagerie モデル導入
- [x] シーン構築(テーブル, スライダ, プッシャチップ, 接触パラメータ)
- [x] Hogan 2016 FOM ベース MPC コントローラ実装
- [x] 統合シミュレーション: y=0.2→0.8 直線プッシュ動作確認
- [x] 動画出力 + データ保存パイプライン(`results/` ディレクトリ構造)
- [x] 座標系整理: base link y 軸方向の符号を `run_stage1.py` に文書化. base quat は維持, 制御は全てワールド座標で動作
- [x] ロボットベースをテーブル上 (z=0.3) に再配置, キーフレーム IK 再計算
- [x] Stage 0 と Δθ 傾向の突き合わせ (`experiments/exp_dtheta_comparison.py`)
- [x] CoM オフセット条件 A/B/C の MuJoCo 上での対照実験 (`experiments/exp_com_abc.py`)
- [x] 接触パラメータ感度分析 (`experiments/exp_contact_sensitivity.py`)

## Stage 1+: 拡張

- [x] MPC 制御条件での軌道追従評価 (`experiments/exp_tracking_eval.py`)
- [x] 圧力分布非一様性の影響検証 (`experiments/exp_pressure_distribution.py`)
- [x] 準静的条件の限界速度調査 (`experiments/exp_speed_limit.py`)

## Stage 2: Robotiq 2F-85 グリッパ統合

- [x] Robotiq 2F-85 XML モデル統合 (`ur5e_with_gripper.xml`)
- [x] フランジ接合・マウント回転 (R_x(-90°)) 調整
- [x] 衝突モデル修正 (グリッパリンケージ-テーブル除外)
- [x] 制御パス更新 (`run_stage1.py`: gripper_pinch サイト, パッドジオメトリ, グリッパアクチュエータ)
- [x] キーフレーム再計算 (`compute_keyframe.py`)
- [x] グリッパ指先によるスライダ押し動作確認 (y=0.78 到達)
## Stage 2 改訂: 鉛直 pusher 化 + 作業板 (2026-06-04)

- [x] base フレームを world 整列に再定義 (`ur5e_with_gripper.xml` quat `0 0 0 -1`→`1 0 0 0`). base +y = 押し方向 (world +y)
- [x] 作業板 (work surface) 追加: base y [0.35, 0.95], 半幅 0.325m, 厚さ 27mm. 支持のためテーブルを +y 拡張
- [x] スライダを作業板上 base (0, 0.4) = world (0, 0.5, 0.342) に配置 (slider×work_surface 接触ペア + グリッパ×work_surface exclude 追加)
- [x] tool0 鉛直下向き (base +x=tool0+x, base −z=tool0+z) の 6-DOF 姿勢 IK で keyframe 生成 (指開閉軸 ⊥ 押し方向で対称接触, 重力下垂 0mm)
- [x] 実行層に tool0 鉛直保持を追加 (`run_stage1.py` approach + MPC ループの 6-DOF IK). MPC (Hogan) 本体は不変
- [x] 鉛直 pusher での 2D 押し成功: base (0,0.4)→(0,0.8) 到達, 接触維持, tool0 傾き 0.03°, 暴走・机貫通なし
- [x] ゴールマーカー (赤) を作業板上面に追加
- [x] 標準実験動画フォーマット確立 (`render_grid_video.py`, 2x2 グリッド overview/top/front/side)
- [x] 残ドリフト抑制 (2026-06-04): x −70.8mm→−1.5mm, θ 13.65°→−0.01°. 真因は 2 点 — (1) MPC に渡す接線接触座標 px_body が pinch 経由で `sin(θ)·法線スタンドオフ` に汚染され, わずかな偏心が straight push に +θ トルクを誤予測させ θ>5° で QP が u=0 に collapse → 対称押しの意図通り px_body=0 を MPC に供給. (2) keyframe が pad 前面を 10mm 事前貫入し settle でスライダを 11cm ラム → 再接触が斜めになり θ スパイク → keyframe を後退 (pinch 0.469→0.452) して貫入解消
- [x] `compute_keyframe.py` を 6-DOF tool-down IK 生成に更新 (2026-06-04): `solve_ik` を位置のみ→位置+tool0 姿勢の 6-DOF 化. `run_stage1` の `get_jacobian6` / `orientation_error` を共有し鉛直 pusher 規約を一元化. ターゲットを retract 位置 (pinch world y=0.452, pad 前面がスライダ面 0.470 から 7mm クリア) に更新. 出力 qpos はデプロイ済み keyframe と完全一致 (arm `1.1803 -1.6653 2.4917 -2.3972 -1.5708 -0.3905`, ori_err=0)
- [x] keyframe 生成器の ctrl とデプロイ XML を整合 (2026-06-04): XML keyframe の ctrl を重力補償値に揃え, 再シムで結果不変 (x −3.6mm, θ −0.01°, ゴール到達) を確認

## リポジトリリファクタ: フラット → モジュラーパッケージ (2026-06-04)

- [x] `pusher_slider/` import パッケージ化 + `pyproject.toml` で editable install. 全 `/workspace` ハードコード (27 箇所) を `paths` モジュールのリポジトリ相対解決に置換
- [x] 機能分離: config / kinematics / io / controllers(Protocol+registry) / sim(runner, keyframe) / analytical / viz(grid_video, plots)
- [x] 設定の CLI 化: `tyro` でネスト dataclass `SimConfig` から CLI 自動生成 (`scripts/run_push.py --push.y-goal ...`). run ごとに `config.json` を dump
- [x] XML を `scenes/` へ移設 (旧 pusher scene は `scenes/legacy/`), meshdir を相対修正
- [x] experiments 6 本を新パッケージ API へ移行 (物理=各自の scene は温存)
- [x] results 整理: grid 動画の無い run + exp_*/ を削除 (1.1GB→87MB), stale な verify_*.png 除去
- [x] 検証: 新 CLI で full sim が現行同等結果, keyframe 再現, grid 動画生成, pytest pass, ruff clean
- [x] README をプロジェクト向けに全面書き換え (クローン→コンテナ→`setup_assets.sh`→push 手順)

## FT300-S + カスタムフィンガー統合

- [x] FT300-S STEP からケーブル除外メッシュ生成 (OCP) → STL エクスポート
- [x] FT300-S body を `wrist_3_link` と `gripper` の間に挿入
- [x] 接触除外・キーフレーム再計算・シミュレーション検証
- [ ] ラウンドフィンガー (`tip;round,finger.stl`) のグリッパーへの取り付け
  - Onshape Y-up → MuJoCo Z-up 座標変換の解決
  - MuJoCo mesh geom の自動 pos/quat 書き換えへの対処

## MPC の論文忠実度 (2026-06-04)

- [x] Hogan WAFR2016 本文と `controllers/mpc.py` を突き合わせ, 各部に論文の節・式番号を注釈
- [x] `_motion_cone` の式バグ (Eq.(2)(3) に対し px²/py² 入替・cross 符号反転) を修正・検証
      (cone が 0 を跨ぐ, full sim 回帰なし, off-center 許容偏心 ~2 倍)
- [ ] off-center 接触+回転での MPC collapse 解消 (公称軌道まわりの線形化 A_j(t),B_j(t) 実装; ISSUES 参照)
