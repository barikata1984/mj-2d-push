# ISSUES

## off-center 接触 + 回転で MPC が u=0 に collapse (緩和済み・残存)

スライダの接触点が中心から大きくずれ (接線オフセット ~1cm), かつ θ がある程度
(≳5°) あると, MPC の QP が押し速度 vn=0 に collapse する (押すと θ が悪化すると予測し,
θ を重く罰するコストが無作為より u=0 を選ぶ).

- 2026-06-04 に `_motion_cone` の式バグ (Hogan Eq.(2)(3) に対し px²/py² 入替・cross 符号反転)
  を修正. これにより許容偏心が ~2 倍に拡大 (collapse 境界 px≈0.004→≈0.01). ただし
  **cone バグは寄与因子であって単独原因ではなく**, 修正後も大偏心+回転では collapse が残る.
- 残存 collapse の主因は別 (おそらく予測の A=0 線形化簡略化 + コスト重み).
- 現状は実行層 (`sim/runner.py`) が中心化接触 `[0.0, -0.03]` を MPC に渡すことで回避.
  グリッパが機械的に対称で接触が中心という前提なので, この対処自体は妥当.
- 解消するなら: 公称軌道まわりの線形化 (A_j(t),B_j(t); paper §4.6) を実装し,
  off-center 接触を復元して `experiments/exp_tracking_eval.py` で再評価する.

## ラウンドフィンガー STL の MuJoCo 取り付けで座標系・原点がずれる

Onshape でエクスポートした `tip;round,finger.stl` を MuJoCo のグリッパーに取り付ける際, 2 つの問題が発生:

1. **座標系流儀の不一致**: Onshape は Y-up (OpenGL 流儀), MuJoCo/ROS は Z-up. STL 頂点に Y-up が焼き付いているため, body に quat="0.7071 -0.7071 0 0" (Rx(-90°)) の補正が必要
2. **MuJoCo mesh geom の自動フレーム調整**: メッシュ geom をコンパイルする際, 凸包の重心 + 主慣性軸に合わせて `geom_pos` / `geom_quat` が自動書き換えされる (`mesh_pos` / `mesh_quat` に保存). XML で `pos="0 0 0"` と書いても, コンパイル後は数 mm のオフセットと ~90° の回転が加わる. これにより CAD 上の原点と MuJoCo 上の geom 原点が一致しない

対処候補:
- Onshape エクスポート時に mate connector で Z-up に合わせてから STL 出力
- geom に逆 pos/quat を設定して自動調整を打ち消す (検証済み: 姿勢は打ち消し可能, 位置も加算的に打ち消し可能)
- MuJoCo の `<compiler>` オプションで自動調整を無効化できるか調査

## MuJoCo 接触ソルバ由来の θ 振動

スライダの θ に ±0.3° 程度の高周波振動が観測される. 準静的解析(Stage 0)では θ は滑らかに推移するため, これは MuJoCo の接触力解法に起因する.

接触パラメータ感度分析 (`experiments/exp_contact_sensitivity.py`) の結果:
- `solref`/`solimp` の調整効果は限定的
- **elliptic cone が必須** — pyramidal cone では振動 42° に達しプッシュ失敗
- `mu_pusher` = 0.1 で振動 2.8° に増大 (デフォルト 0.3 で ~0.5°)
- 現在の設定 (elliptic, µ=0.3, solref=0.02) は安定動作の最適近傍

→ 振幅 ±0.3° は本系のソルバ残差として許容可能. 定量比較では平滑化フィルタを適用すること.
