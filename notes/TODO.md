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
- [ ] 座標系整理: base link y 軸方向の符号(現在ワールド +y, base -y)
- [ ] Stage 0 と Δθ 傾向の突き合わせ
- [ ] CoM オフセット条件 A/B/C の MuJoCo 上での対照実験
- [ ] 接触パラメータ感度分析(`solref`/`solimp`/`friction`)

## Stage 1+: 拡張

- [ ] MPC 制御条件での軌道追従評価(RMS 誤差, 制御努力)
- [ ] 圧力分布非一様性の影響検証
- [ ] 準静的条件の限界速度調査
