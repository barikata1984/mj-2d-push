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
- [ ] MPC 接触モデル調整 (x ドリフト・θ 回転の改善)
- [ ] グリッパパッド面の接触最適化 (フォロワーメッシュ依存からの脱却)
