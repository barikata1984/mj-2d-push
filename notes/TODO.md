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
- [ ] `compute_keyframe.py` を 6-DOF tool-down IK 生成に更新 (現在は位置のみ. 2026-06-04 の retract keyframe は `run_stage1` の IK を流用したインライン script で再計算したもので, compute_keyframe.py 自体は依然非同期)
