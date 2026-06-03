---
Title: "Feedback Control of the Pusher-Slider System: A Story of Hybrid and Underactuated Contact Dynamics"
Authors:
  - Hogan, François Robert
  - Rodriguez, Alberto
Year: 2016
Venue: WAFR
Tags:
  - "pusher-slider"
  - "model-predictive-control"
  - "hybrid-dynamics"
  - "contact-control"
  - "nonprehensile-manipulation"
PDF: "[[papers/Hogan-WAFR2016-Feedback_Control_Pusher-Slider/main.pdf|📃]]"
Import Date: "2026-06-03"
Read Date: 2026-06-03
Executive Summary: "準静的 pusher-slider 系に対し, 固着・上滑り・下滑りの3接触モードを持つハイブリッド動力学を混合整数二次計画(MIQP)として定式化し, MPC で閉ループ制御する手法を提案した. 指数的に分岐するモードスケジュール問題を, 少数の代表的モード列を事前選定する Family of Modes (FOM) で凸 QP に帰着させリアルタイム化を実現. ABB IRB 120 ロボットで直線追従・目標追跡タスクを実験検証した."
Citekey: Hogan-WAFR2016-Feedback_Control_Pusher-Slider
BibTeX Key: hogan2016feedback
DOI: ""
Relevance: 5
Repository: none
Category: note
Template Version: v2.3
---

## Executive Summary
準静的 pusher-slider 系に対し, 固着・上滑り・下滑りの3接触モードを持つハイブリッド動力学を混合整数二次計画(MIQP)として定式化し, MPC で閉ループ制御する手法を提案した. 指数的に分岐するモードスケジュール問題を, 少数の代表的モード列を事前選定する Family of Modes (FOM) で凸 QP に帰着させリアルタイム化を実現. ABB IRB 120 ロボットで直線追従・目標追跡タスクを実験検証した.

---
## Summary

### この論文が答えた問い, あるいは解決した課題は何か?

摩擦接触を介したロボットマニピュレーション系において, (1) 接触モードの離散的切替(ハイブリッド性)と (2) 摩擦錐による入力制約(劣駆動性)の両方を扱えるリアルタイムフィードバック制御器をどのように設計するか, という問題に取り組んだ. 具体的には, 単点プッシャによる平面スライダの姿勢制御を対象としている.

### 提案手法のアプローチと, その根幹をなす要素は何か?

楕円体限界曲面近似の下で pusher-slider の準静的運動方程式を導出し, 名目軌道まわりに線形化した上で, モード依存の線形拘束付き MPC を構成する. ハイブリッド性は整数変数で, 劣駆動性はモーションコーン拘束で表現し, これらを統一的に最適化問題として解く.

- **楕円体限界曲面**(Howe & Cutkosky 1996 の近似): 摩擦力 f_max = µ_g m g, 摩擦モーメント m_max = µ_g m g (1/A) ∫ |r_dm| dA. 比 c = m_max/f_max が運動方程式の中心パラメータ(§4.3)
- **モーションコーン**: 接触点における固着条件を満たすプッシャ速度の集合. 上限角 γ_t と下限角 γ_b は c, 接触位置 (p_x, p_y), 摩擦係数 µ から決まる(§4.4, Eqs. 2–3)
- **ハイブリッド運動方程式**: 3モード(固着 j=1, 上滑り j=2, 下滑り j=3)ごとに異なる線形行列 (A_j, B_j) を持つ区分線形系(§4.5, Eq. 8)
- **MIQP 定式化**: big-M 法で整数変数 z_jn ∈ {0,1} によりモード依存拘束を活性化/非活性化(§5.1)
- **Family of Modes (FOM)**: 3^N 個のモードスケジュールから物理的直感に基づき少数(m=3)の代表列を事前選定し, m 個の凸 QP の最小値を取る. リアルタイム実行可能(§5.2)

### 特に参考とした既存研究と, それらと比した提案手法の新規性は何か?

- **[[papers/Lynch-IJRR1996-Stable_Pushing_Mechanics/stable-pushing-mechanics-controllability-and-planning|Lynch & Mason 1996]]**: 安定プッシュによるセンサレス制御. 本論文はセンサフィードバックを前提とし, 安定プッシュ条件外(滑り接触)も積極的に利用する点で拡張
- **Lynch (1992)**: pusher-slider の運動方程式の原型. 本論文はこれを直接利用し MPC に組み込む
- **Posa, Cantu, Tedrake (2014)**: 接触力を決定変数とする軌道最適化. オフライン計画であり, 本論文のリアルタイムフィードバックとは対照的
- **Tassa & Todorov (2010, 2012)**: 平滑化接触モデルによる最適制御. 本論文はハイブリッド性を陽に扱う点で異なるアプローチ

新規性は, (1) pusher-slider のハイブリッド接触動力学を MIQP–MPC として定式化した最初の研究であること, (2) FOM による実用的なリアルタイム化手法を提案したことにある.

### どのように訓練・最適化したのか?

N/A: 学習を含まないモデルベース制御.

- **損失関数 / 最適化目的**: MPC の有限ホライズンコスト J = x̄_N^T Q_N x̄_N + Σ(x̄_{n+1}^T Q x̄_{n+1} + ū_n^T R ū_n). Q = 10 diag{1, 3, 0.1, 0}, Q_N = 200 Q, R = I(§5, §6.1)
- **データセット**: N/A. シミュレーション + 実機実験

### どのように検証したか? 指標と結果は?

2つのシナリオでシミュレーションと実機実験の両方を実施(§6):

**シナリオ 1: 直線追従(§6.1)**
- 名目軌道: x*(t) = [0.05t, 0, 0, 0]^T(等速直線)
- 外乱: 横方向インパルス + 15° 回転擾乱
- 結果: シミュレーション・実機ともに擾乱から名目軌道へ収束. 定量的誤差値は未掲載だが, Fig. 8 で視覚的に確認可能

**シナリオ 2: 目標追跡(§6.2)**
- 3つの連続ターゲットを追跡. 到達許容距離 0.01 m
- 結果: シミュレーション・実機ともに全3ターゲットに到達. 実機では旋回がシミュレーションよりやや控えめ(摩擦モデルの不一致によると著者は推測)

物理パラメータ(Table 1): µ_p = 0.3, µ_g = 0.35, m = 1.05 kg, スライダ 0.09 × 0.09 m. MPC パラメータ: N = 35 ステップ, h = 0.03 s, |v_n|, |v_t| ≤ 0.1 m/s.

### 検証結果に基づいた議論, 明らかになった課題はあるか?

(§7 より)

- FOM のモード列選定は物理的直感に依存しており, より複雑な操作タスク(多接触点・把持内操作)への拡張は自明ではない. Extrinsic dexterity タスクへの適用を今後の方向として挙げている
- シミュレーションと実機の旋回挙動の差異が観測されているが, 原因の詳細な分析は行われていない
- 著者は限界に明示的には言及していないが, 均一圧力分布仮定, 準静的仮定の適用範囲, 分離モードの非考慮は暗黙の制約として存在する

---
## 自身の研究との関連

本論文は Stage 1 MuJoCo 実装と, その先の MPC 制御条件(session_summary §3.4"応用指標")の直接的な参照先である.

1. **運動方程式の直接利用**: §4.5 の Eq. 8 は session_summary §3.2 の閉形式と等価(固着モード j=1 の場合). Stage 0 シミュレータの理論的出典. 行列 Q の中で c² = (m_max/f_max)² が現れ, CoM が (p_x, p_y) を通じて運動に入る構造が明示されている
2. **c パラメータと CoM の関係**: c = m_max/f_max は接触点 p を CoM 基準で定義するため, CoM 誤差は p の系統的バイアスとなり, モーションコーン境界 γ_t, γ_b の誤りを通じて制御性能に影響する. これが Δθ 指標の力学的根拠
3. **MPC 制御条件への展開**: session_summary §3.4 の"MPC 軌道追従での最終姿勢誤差・追従 RMS・制御努力"は本論文の §6 の実験構成をそのまま応用可能. FOM の3モード列(M1: 上滑り→固着, M2: 下滑り→固着, M3: 固着のみ)をベースに, 条件 A/B/C の CoM 仮定を入れ替える対照実験を構成できる
4. **Stage 1 実装上の注意**: Table 1 の物理パラメータ(µ_p, µ_g, 質量, 寸法)は MuJoCo モデルの初期値として参考になる. ただし MuJoCo の接触ソルバは本論文の楕円体限界曲面近似とは異なる接触力学を解くため, c の実効値にモデル不一致が生じる(session_summary §4.4 の指摘と一致)

---
## 追加議論


---
## BibTex
<details>
<summary> Click to show/noshow the BibTex data </summary>

```bibtex
@inproceedings{hogan2016feedback,
  author    = {Hogan, Fran{\c{c}}ois Robert and Rodriguez, Alberto},
  title     = {Feedback Control of the Pusher-Slider System: A Story of Hybrid and Underactuated Contact Dynamics},
  booktitle = {Proceedings of the 12th International Workshop on the Algorithmic Foundations of Robotics (WAFR)},
  year      = {2016},
  address   = {San Francisco, CA, USA},
  note      = {arXiv:1611.08268},
}
```
</details>
