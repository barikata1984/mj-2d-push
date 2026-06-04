# ISSUES

## MuJoCo 接触ソルバ由来の θ 振動

スライダの θ に ±0.3° 程度の高周波振動が観測される. 準静的解析(Stage 0)では θ は滑らかに推移するため, これは MuJoCo の接触力解法に起因する.

接触パラメータ感度分析 (`experiments/exp_contact_sensitivity.py`) の結果:
- `solref`/`solimp` の調整効果は限定的
- **elliptic cone が必須** — pyramidal cone では振動 42° に達しプッシュ失敗
- `mu_pusher` = 0.1 で振動 2.8° に増大 (デフォルト 0.3 で ~0.5°)
- 現在の設定 (elliptic, µ=0.3, solref=0.02) は安定動作の最適近傍

→ 振幅 ±0.3° は本系のソルバ残差として許容可能. 定量比較では平滑化フィルタを適用すること.
