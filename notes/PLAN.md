# PLAN

## 実験の段階構成

session_summary.md §3.7 のラダーに従う:

1. **2D プッシュ**(CoM_xy + c の整合) ← 現在ここ
2. 質量が効く課題(力計測 or 加速を伴う押下/持ち上げ)
3. MoI が効く課題(回転動力学・スピン・動的プッシュ)

## Stage 1 の二段構え

- **Stage 0**(完了): 自前解析 LS シミュレータ. CoM がモデルに入る構造を完全制御. 実験設計の妥当性確認済み
- **Stage 1**(進行中): MuJoCo. 接触ソルバの現実性・モデル不一致を検証

## Stage 1 の次のマイルストン

1. Stage 0 と MuJoCo で Δθ 傾向を突き合わせ(無ノイズ, 同一パラメータ)
2. 一致を確認後, CoM オフセット条件 A/B/C を MuJoCo 上で実施
3. `estimate_com` と `induced_rotation` の入力を MuJoCo 観測 twist に差し替え(LS モデルは制御側に残し, 物理のみ MuJoCo へ置換)
