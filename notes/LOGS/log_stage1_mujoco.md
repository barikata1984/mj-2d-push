# Stage 1: MuJoCo pusher-slider シミュレーション

## 2026-06-03: 初期実装完了

### 環境構築

- MuJoCo 3.8.1 を pixi 経由で導入
- UR5e モデルは mujoco_menagerie (google-deepmind) から sparse clone
- ffmpeg, Pillow を追加(動画レンダリング用)

### シーン設計 (`stage1_scene.xml`)

- テーブル高さ z=0.3m(z=0 だと UR5e の重力補償が不足し腕がたわむ)
- スライダ: 80x60x30mm, m=1.05kg, condim=4(ねじれ摩擦有効)
- 摩擦パラメータ: µ_pusher=0.3, µ_ground=0.35 (Hogan 2016 Table 1 準拠)
- プッシャ: wrist_3_link の子ボディとして 5mm 球体チップを追加
- 接触ペア明示指定, ロボットアーム各リンクとテーブル/スライダの接触除外

### MPC コントローラ (`pusher_slider_mpc.py`)

- Hogan & Rodriguez 2016 の Family of Modes (FOM) アプローチ
- 3モードスケジュール(M1: 上滑り→固着, M2: 下滑り→固着, M3: 固着のみ)
- 各スケジュールで凸 QP を scipy SLSQP で解き, 最小コストを選択
- 準静的仮定により A=0, 予測は x_k = x_0 + dt * Σ(B_j * u_j)
- 平均 MPC 計算時間: ~6-9ms/ステップ

### ロボット制御

- PD 位置制御アクチュエータに対し, ヤコビアン擬似逆行列経由で ctrl を蓄積
- ctrl += J_pinv @ (v_desired * dt) で重力サグを暗黙補償
- z 方向は比例制御(gain=5)でテーブル面高さを維持

### 結果

| 指標 | 値 |
|---|---|
| 走行距離 | y: 0.235 → 0.781 (55cm) |
| x ドリフト | 8.1mm |
| θ 最終誤差 | 0.16° |
| シミュレーション時間 | 7.11s |
| MPC ステップ数 | 234 |

### 出力パイプライン

- `results/<trial_id>/pics/`: 各フレーム PNG
- `results/<trial_id>/push_sim.mp4`: ffmpeg エンコード動画(side カメラ)
- `results/<trial_id>/data.npz`: 17 系列 × 235 ステップ(slider pose, pusher pos, 接触力, 関節状態等)
- `results/<trial_id>/config.json`: パラメータ記録
- `results/<trial_id>/result.png`: 6 パネルサマリプロット

### 既知の課題

- ワールド +y 方向にプッシュしているが, UR5e base body の quat="0 0 0 -1" により base link y 軸はワールド -y. 符号の整理が必要
- θ に ±0.3° 程度の振動あり(MuJoCo 接触ソルバ由来)
- Stage 0 との Δθ 傾向突き合わせは未実施

## 2026-06-03: 論文サマリ作成

- Lynch & Mason, IJRR 1996 "Stable Pushing" のサマリノート作成
- Hogan & Rodriguez, WAFR 2016 "Feedback Control of Pusher-Slider" のサマリノート作成
- 配置先: `literature/papers/{citekey}/`
