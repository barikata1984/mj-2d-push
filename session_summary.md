# 2D プッシュによる慣性(重心)推定の実験設計 — セッションまとめ

## 0. 全体の流れ

1. ロボットマニピュレーションにおける 2D プッシュの基本方針(状態組込み制御 vs 開ループ+再プランニング)の整理
2. 非学習ベースの 2D プッシュ研究の網羅的サーベイ
3. 慣性推定の有効性を示す実験として 2D プッシュを使う設計(推定 CoM vs 幾何中心の対照)
4. 「面内 CoM チャンネルの検証」というタスク位置づけの確認
5. シミュレータ選定と Stage-0 実装・動作確認

---

## 1. プッシュ制御の主流アプローチ

**結論:物体姿勢を状態に組み込み、(多くは閉ループの)制御・計画問題として解くのが圧倒的に主流。**

理由:プッシュは非把持(non-prehensile)・劣駆動で、物体運動が不確実。
- 物体と床面の圧力分布が未知 → 摩擦が空間的にばらつく
- 接触点と物体姿勢の関係で物体が回転・スリップする

したがって「ゴールへ一直線に押す」だけでは狙いどおりに進まない。物体姿勢 SE(2) = (x, y, θ) が運動を決める支配変数なので、状態に入れるのが必然。

**二択はややミスリード**:両アプローチとも物体姿勢をセンサで追う点は共通。違いは補正タイミング。
- 主流:連続的な閉ループ(MPC / receding-horizon / RL 方策)で毎ステップ補正
- 「一直線+失敗検知で中断・全体再計画」:誤差が蓄積するため脆弱で、独立戦略としては非主流

---

## 2. 非学習ベース 2D プッシュ研究のサーベイ

> 専用サーベイ: Stüber, Zito, Stolkin, *"Let's Push Things Forward: A Survey on Robot Pushing"*

3層構造で整理できる。

### 2.1 力学層(基礎理論の3つの柱)

| 概念 | 出典 | 内容 |
|---|---|---|
| 押下力学・モーションコーン | Mason 1986 | 接触点・摩擦が回転方向を決める voting theorem。固着を保って生成できる物体運動集合 = モーションコーン |
| 限界曲面 (Limit Surface) | Goyal, Ruina, Papadopoulos 1989, 1991 | 古典塑性論由来。摩擦界面が支えうる全摩擦力・モーメント集合の境界。滑り運動と摩擦荷重の完全な関係を内包。実用上は楕円体近似(Howe & Cutkosky 1996) |
| 一般化摩擦錐 | Erdmann 1994 | 接触で伝達可能な力の集合 |

準静的(quasi-static)仮定が前提:押下が遅く慣性力が無視でき、押すのをやめれば物体も止まる。

### 2.2 計画層

- **安定プッシュ** (Lynch & Mason 1996):非ホロノミック速度拘束、可制御性、障害物中の安定プッシュ経路プランナ
- **逐次プッシュによる姿勢決定**:Mason 1990、Akella & Mason 1992/1998 ほか
- **センサレス操作・ファネル**:Erdmann & Mason、Goldberg "Orienting Polygonal Parts without Sensors"、Brost の push diagram。フィードバックなしで初期不確実性を吸収
- **クラッタ中の rearrangement**:Dogar & Srinivasa。押下で不確実性をファネル

### 2.3 制御層(閉ループ)

- **触覚フィードバック制御**:限界曲面モデルで運動方程式を導出、能動センシング(重心)
- **ハイブリッド MPC** (Hogan & Rodriguez 2016–2018):pusher-slider を固着/左滑り/右滑り/非接触の離散モードを持つハイブリッド系として整数計画 + MPC。"Reactive Planar Manipulation with Convex Hybrid MPC" でモード列をオフライン分離してリアルタイム化
- **微分平坦性** (Zhou, Hou, Mason 2019)
- **補完性制約付き軌道最適化 (MPCC)**:固着・滑りを切替

### 2.4 状態推定層(両者をつなぐ)

- Shape and pose recovery from planar pushing (Yu, Leonard, Rodriguez 2015) — バッチ SLAM
- Realtime State Estimation with Tactile and Visual Sensing (Yu & Rodriguez 2018) — ファクタグラフ
- Tactile SLAM (Suresh, Bauza et al. 2021) — GPIS + ファクタグラフのオンライン SLAM

### 2.5 周辺・実験基盤

- データセット:Yu, Bauza, Fazeli, Rodriguez *"More than a million ways to be pushed"* (IROS 2016)
- 摩擦モデリング精緻化:friction patches (Ardakani 2020)、LuGre + 限界曲面 (2024)
- 拡張:マルチロボット協調押下、object-centric kinodynamic planning

---

## 3. 慣性(重心)推定実験の設計

### 3.1 最重要の前提:準静的プッシュで観測できる量

準静的では運動方程式に加速度項が入らず、**質量 m と慣性モーメント I は運動に現れない**。
効くのは圧力分布の重心(= 面内 CoM 投影)と摩擦のみ。

- **CoM 推定の有効性を示す** → 2D プッシュは理想的(CoM は運動予測に直接効く=可識別)
- **完全な慣性テンソル(m, I)を推定** → 準静的では原理的に観測不可。準動的/動的レジーム、または力トルク計測が必要

ユーザーの方針:慣性パラメータ全推定は可能だが、**CoM の寄与が見えるタスクから始める**ため 2D プッシュを最初に置く。これは「面内 CoM チャンネル」だけを m・I から復号して独立検証する妥当な段取り。

- 検証されるのは **CoM の (x, y) 成分**(平面上の物体では中心圧 ≈ 3D CoM の鉛直投影)
- CoM の z 成分とフル MoI テンソルはプッシュでは触れない → 主張は「面内 CoM 検証」に限定
- 限界曲面比 c (≈ 圧力分布の二次モーメント、MoI ではない)が副次的に得られ、弱い整合性チェックに使える

### 3.2 制御モデル(楕円体限界曲面の pusher-slider)

物体座標系を CoM 原点に取り、接触点 r=(pₓ, p_y)、接触力 f=(fₓ, f_y):

- 重心まわりモーメント: `m_z = pₓ·f_y − p_y·fₓ`
- 楕円体限界曲面: `H = (fₓ/f_max)² + (f_y/f_max)² + (m_z/m_max)² = 1`
- 物体 twist は法線方向: `[vₓ, v_y, ω] ∝ [fₓ, f_y, m_z/c²]`,  `c = m_max/f_max`(長さ次元)

**対照の核心**:`m_z` は CoM まわりのモーメント。幾何中心を原点に取ると moment arm が誤り、ω 予測が系統的にバイアスする。

固着接触の閉形式(本セッションで導出):

```
ω = (pₓ·v_y − p_y·vₓ) / c²
[vp_x; vp_y] = M [vₓ; v_y],  M = [[1 + p_y²/c², −pₓp_y/c²],
                                  [−pₓp_y/c², 1 + pₓ²/c²]]
```

純並進(ω=0)は (vₓ, v_y) ∥ (pₓ, p_y) のとき、すなわち**CoM を通る線上を押すとき**に起こる。誤った CoM 仮定 → d に比例した回転。

### 3.3 実験セットアップ

- **独立変数**:CoM オフセット d(幾何中心からのずれ)。d=0 では差が出ないので d を振ることが主軸
- **物体**:均質平板 + 既知おもりで d を制御。真 CoM は天秤/CAD で確定
- **ハードウェア**:平滑・均一摩擦テーブル、真上から AprilTag 等で姿勢追跡、単点(または平面フェンス)プッシャ、準静的低速

### 3.4 制御条件と評価

| 条件 | 仮定する CoM |
|---|---|
| A(ベースライン) | 幾何中心 |
| B(処置) | 推定 CoM |
| C(上界) | 真 CoM |

**重要**:強い閉ループ制御は悪いモデルをフィードバックで補償し差を消す。推定の価値を見せるには**開ループ or 弱フィードバック**で露出させる(第1ターンの開ループ方式は運用上脆弱でも、モデル品質プローブとしては理想的)。

**主指標**:純並進を狙った直進押下で誘起される回転 **Δθ**。
- 真 CoM 線上 → Δθ ≈ 0
- 幾何中心を誤認 → Δθ ∝ d
- 予測:Δθ(A) は d に比例して大、Δθ(B) は推定が良ければ小、Δθ(C) が残差モデル誤差の床

応用指標:MPC 軌道追従での最終姿勢誤差・追従 RMS・制御努力。

### 3.5 CoM 推定法

数回の探索押下で (押下入力, 観測 twist) を集め、楕円体モデルで「回転を最もよく説明する CoM」を最小二乗/ファクタグラフで同定。**プッシュから CoM を推定し、その CoM でプッシュ制御を改善する**自己完結ループ。

### 3.6 落とし穴

- 圧力分布の非一様・床摩擦の空間変動が推定をバイアス(c との交絡も)
- 真 CoM の ground truth が必要
- 準静的を保つため低速で(将来の m・I 段では逆に動的レジームを利用)
- 単点接触 + 固着/滑りが最もクリーン

### 3.7 タスクの段階づけ(ラダー)

1. **2D プッシュ**(CoM_xy、+c の整合)← 最初
2. 質量が効く課題(力計測 or 加速を伴う押下/持ち上げ)
3. MoI が効く課題(回転動力学・スピン・動的プッシュ)

---

## 4. シミュレータ選定と実装

### 4.1 二段構え

- **Stage 0(まず)**:自前の解析 LS シミュレータ(NumPy)。汎用エンジン不要。CoM がモデルのどこに入るか完全制御、制御モデルと厳密一致。実験設計の妥当性検証に最適
- **Stage 1**:MuJoCo。接触ソルバの現実性・モデル不一致を検証

### 4.2 シミュレータ比較

| 候補 | 評価 |
|---|---|
| **自前解析 LS** | Stage 0 用。最速・完全制御・制御モデルと一致 |
| **MuJoCo** | 推奨。無料・接触安定・`<inertial pos>` で CoM 直接設定・`condim=4` でねじれ摩擦 |
| Drake | 接触力学が厳密、Tedrake 講義が LS プッシュを扱う。学習コスト高 |
| PyBullet | 手軽だが摩擦が粗い。定性確認向け |
| Isaac | 将来 RL・多環境並列向け |

### 4.3 MuJoCo 最小構成

- 床:`plane`(`friction` 明示)
- スライダ:`box` + `<inertial pos="dx dy 0" mass=".." diaginertia=".."/>`(CoM オフセット = d)
- プッシャ:小 `cylinder` を位置/速度制御
- 接触:`condim=4`、平板底面四隅接触で「CoM 偏り → 法線力非対称 → 摩擦トルク」

### 4.4 Stage 1 で初めて見える交絡(現実性チェックの本体)

- 圧力分布が静定不能 → ソルバ依存。接触パラメータ(`solref`/`solimp`/`friction`)を固定・記録
- LS モデル ≠ MuJoCo 接触 → C(真 CoM)でも Δθ の床が Stage 0 より上がる。この残差が「モデル不一致ロバスト性」の定量値
- 速度を上げすぎると準静的が崩れ慣性混入

### 4.5 Stage-0 実装結果

`push_com_sim.py` の実行結果(理論予測どおり):

| d [mm] | CoM 推定誤差 [mm] | Δθ A(幾何) [deg] | Δθ B(推定) [deg] | Δθ C(真) [deg] |
|---:|---:|---:|---:|---:|
| 0.0  | 0.73 | 0.00 | 0.02 | 0.00 |
| 5.0  | 0.71 | 1.26 | 0.01 | 0.00 |
| 10.0 | 0.69 | 2.48 | 0.00 | 0.00 |
| 15.0 | 0.68 | 3.62 | 0.01 | 0.00 |
| 20.0 | 0.66 | 4.65 | 0.02 | 0.00 |
| 25.0 | 0.64 | 5.56 | 0.02 | 0.00 |
| 30.0 | 0.62 | 6.33 | 0.03 | 0.00 |

**A ≫ B ≈ C** の綺麗な対照が成立。Δθ が CoM 誤差の直接的可視化指標として機能し、実験設計の可識別性・指標感度が確認できた。

---

## 5. 次の一歩

無ノイズ MuJoCo で Stage 0 と Δθ の傾向を突き合わせる。一致後、`estimate_com` と `induced_rotation` の入力を sim 観測 twist に差し替え(LS モデルは制御側に残し物理のみ MuJoCo へ置換)。

## 主要参考文献

- Mason, *Mechanics and Planning of Manipulator Pushing Operations*, IJRR 1986
- Goyal, Ruina, Papadopoulos, *Planar sliding with dry friction Part 1/2*, Wear 1991
- Lynch & Mason, *Stable Pushing: Mechanics, Controllability, and Planning*, IJRR 1996
- Howe & Cutkosky, *Practical force-motion models for sliding manipulation*, IJRR 1996
- Hogan & Rodriguez, *Feedback Control of the Pusher-Slider System*, WAFR 2016
- Hogan, Grau, Rodriguez, *Reactive Planar Manipulation with Convex Hybrid MPC*, ICRA 2018
- Yu, Leonard, Rodriguez, *Shape and pose recovery from planar pushing*, IROS 2015
- Suresh, Bauza et al., *Tactile SLAM*, ICRA 2021
- Stüber, Zito, Stolkin, *Let's Push Things Forward: A Survey on Robot Pushing*
