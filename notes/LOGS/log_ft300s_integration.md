# FT300-S センサモデルの MuJoCo 統合

## 目的

UR5e のフランジと Robotiq 2F-85 グリッパーの間に Robotiq FT300-S 力覚センサを挟んだ MuJoCo モデルを作成する.

## 現状の構成

- `scenes/stage1_scene.xml` → `scenes/ur5e_with_gripper.xml` を include
- `ur5e_with_gripper.xml`: UR5e + Robotiq 2F-85 を一体定義 (MuJoCo Menagerie ベース)
- メッシュ: `assets/` に UR5e `.obj` + Robotiq `.stl` をシンボリックリンク (`scripts/setup_assets.sh`)
- FT300-S モデルは現時点でプロジェクトに存在しない

## やったこと

1. **ロボット構成の特定**: `stage1_scene.xml` → `ur5e_with_gripper.xml` の 2 段 include 構造, メッシュの出所 (MuJoCo Menagerie の `universal_robots_ur5e` + `robotiq_2f85`) を確認
2. **既存 FT300 リソースの発見**:
   - STEP ファイル: `FTS-300-S_support_20210305-Sep-06-2024-02-41-04-2138-PM.step` (プロジェクトルート + `~/Downloads`)
   - 別プロジェクトに既存メッシュあり: `osx_ros_atsushi_kuno/.../meshes/robotiq_ft300.stl`, `ft300.stl`, `ft300.urdf.xacro`
3. **STEP ファイルのアセンブリ構造調査**: PRODUCT エントリを列挙し, 除外対象サブアセンブリ `G-E05-ASM_CARTE_G_COUPLING_ET_CABLE_1M_Longueur 1m` (ケーブル + カップリング基板) を特定

## 次にやるべきこと

### 1. FT300-S メッシュの生成

STEP ファイルから `G-E05-ASM_CARTE_G_COUPLING_ET_CABLE_1M_Longueur 1m` サブアセンブリを除外し, センサ本体のみを STL にエクスポートする. 方法の候補:

- **FreeCAD** (GUI): STEP を開き, ツリーから該当サブアセンブリを削除 → Part をマージ → STL エクスポート
- **CadQuery / OCP (Python)**: `pixi` 環境に `cadquery` を追加してスクリプトで処理
- **既存 STL の流用**: `osx_ros_atsushi_kuno` の `robotiq_ft300.stl` が使えるか検証 (形状・スケールの確認)

### 2. 慣性パラメータの取得

FT300-S の質量・慣性テンソルを確定する. データシートから:

- 質量: ~0.30 kg
- 外形: 直径 75 mm, 高さ ~34 mm (円盤形状)

### 3. MuJoCo XML への統合

`ur5e_with_gripper.xml` の `wrist_3_link` → `gripper` body の間 (169 行目) に FT300-S body を挿入:

```xml
<!-- wrist_3_link 内, gripper body の直前に挿入 -->
<body name="ft300s" pos="0 0.1 0" quat="...">
  <inertial mass="0.30" pos="0 0 0" diaginertia="..." />
  <geom type="mesh" mesh="ft300s" class="visual" />
  <geom type="cylinder" size="0.0375 0.017" class="collision" />
  <site name="ft300s_sensor" pos="0 0 0" />
  <!-- gripper body をここの子に移動 -->
</body>
```

### 4. 接触除外・キーフレームの更新

- `stage1_scene.xml` の `<contact><exclude>` に `ft300s` vs `table` / `work_surface` を追加
- `keyframe` の `qpos` / `ctrl` 値を再計算 (FT300-S の高さ分だけエンドエフェクタ位置が変わる)

## 実施内容 (2026-06-07)

### 1. FT300-S メッシュの生成

- `pixi add ocp` で OpenCascade Python バインディングを導入
- STEP ファイルを XCAF で読み込み, `G-E05-ASM_CARTE_G_COUPLING_ET_CABLE_1M` サブアセンブリ (ケーブル + カップリング基板) を除外
- 49 コンポーネントを保持した STEP を `assets/ft300s_no_cable.step` (62 MB) に出力
- STL を `assets/ft300s.stl` (7.1 MB, バイナリ, 線形偏差 0.1 mm) にエクスポート
- 単位は mm (BBox: X 96, Y 93, Z 42 mm)

### 2. MuJoCo XML への統合

- `ur5e_with_gripper.xml` に FT300-S メッシュ (`scale="0.001 0.001 0.001"`), マテリアル, body を追加
- `wrist_3_link` と `gripper` の間に `ft300s` body を挿入 (pos="0 0.1 0" quat="1 -1 0 0")
- FT300-S body: mass=0.30 kg, 衝突は cylinder (r=37.5 mm, h=34 mm), ビジュアルは STL メッシュ
- gripper body は ft300s の子に移動 (pos="0 0 0.034")

### 3. 接触除外・キーフレームの更新

- `stage1_scene.xml` に `ft300s` vs `table` / `work_surface` の exclude を追加
- `scripts/make_keyframe.py` で IK 再計算: 全候補で pos_err=0, ori_err=0 に収束
- 重力沈み 0.0 mm, キーフレーム更新済み

### 4. 検証結果

- `test_scene.py` PASS: EEF ドリフト 0.026 mm, スライダドリフト 0.108 mm (10 秒)
- `run_push.py` でプッシュシミュレーション成功: ゴール到達 (y=0.9006), x 誤差 3.5 mm, θ 誤差 0.02°
- シミュレーション時間 7.85 s, MPC 187 ステップ, 平均 13.9 ms/step

### 5. ラウンドフィンガー取り付け (未完了)

`tip;round,finger.stl` のグリッパーへの取り付けを試行したが, 以下の問題で中断:

- MuJoCo がメッシュ geom コンパイル時に凸包重心 + 主慣性軸で `geom_pos` / `geom_quat` を自動書き換えするため, CAD 原点と geom 原点がずれる
- Onshape (Y-up) と MuJoCo (Z-up) の座標系流儀の違いが発覚
- body quat で Y-up→Z-up 回転を適用しても, 自動 `mesh_pos` オフセットが残り位置合わせ困難
- FT300-S 統合はコミットし, フィンガー取り付けは次回に持ち越し
