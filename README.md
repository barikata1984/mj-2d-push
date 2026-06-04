# pusher-slider: UR5e 2D push simulation

UR5e + Robotiq 2F-85 を鉛直 pusher 化し, Hogan-2016 Family-of-Modes MPC で
作業板上のスライダを準静的に 2D プッシュする MuJoCo シミュレーション。

開発環境は NVIDIA CUDA + Ubuntu の VS Code devcontainer で, Python は
[pixi](https://pixi.sh/) が `pixi.lock` でビット単位に再現する。

## クイックスタート (クローン → 実行)

### 0. 前提

- Docker (+ NVIDIA GPU は任意。CPU でも動く)
- 以下のどちらか:
  - **VS Code** + Dev Containers 拡張 (推奨), または
  - ホスト側の **pixi** (CLI からコンテナを操作する場合)

### 1. クローン

```bash
git clone <this-repo-url> pusher-slider
cd pusher-slider
```

### 2. コンテナを起動

**VS Code の場合**: フォルダを開き, コマンドパレット →
`Dev Containers: Reopen in Container`。初回はイメージビルド + `pixi install`
(パッケージの editable install を含む) が自動で走る。

**CLI の場合** (ホストに pixi がある):

```bash
pixi run build     # イメージをビルド
pixi run up        # コンテナを起動 (detached)
pixi run shell     # コンテナ内 zsh に入る
```

pixi を使わない場合は `cd .devcontainer && docker compose up -d --build` でも可。

### 3. メッシュ資産を取得 (コンテナ内で 1 回)

ロボットメッシュは [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie)
由来で, リポジトリには含めていない (gitignore)。初回のみ取得スクリプトを実行する:

```bash
pixi run setup     # menagerie を sparse clone し assets/ にメッシュを symlink
```

### 4. プッシュシミュレーションを実行

```bash
pixi run push                         # デフォルト設定で実行
pixi run render results/<trial>       # 出力された run を 2x2 グリッド動画に
```

`pixi run push` は `results/<UTC タイムスタンプ>/` に `data.npz` /
`config.json` / `push_sim.mp4` / `result.png` を出力する (`results/` は gitignore)。
スライダがゴール (world y=0.9) に到達し, 最終 x・θ がほぼ 0 になれば成功。

## 設定の変更 (CLI)

設定は `tyro` がネスト dataclass `SimConfig` から CLI を自動生成する。
全オプションは `--help` で一覧できる:

```bash
pixi run python scripts/run_push.py --help
pixi run python scripts/run_push.py --push.y-goal 0.85 --mpc.v-max 0.1 --render.camera top
```

その他のエントリ:

```bash
pixi run keyframe       # 'ready' keyframe を 6-DOF tool-down IK で再計算
pixi run analytical     # Stage-0 解析 (limit-surface) 実験
```

## プロジェクト構成

```
pusher_slider/            # import パッケージ (pip install -e . 済み)
├── paths.py              # repo 相対の scene / asset / results 解決
├── config.py             # tyro 用ネスト dataclass (SimConfig: push/mpc/robot/render)
├── kinematics.py         # 共有 IK ヘルパ (Jacobian, 姿勢誤差, R_TOOL0_DES)
├── io.py                 # trial ディレクトリ, config dump, 時系列 Log/npz
├── controllers/          # コントローラ + レジストリ (make_controller)
│   ├── base.py           #   Controller Protocol
│   └── mpc.py            #   PusherSliderMPC (Hogan 2016 FOM)
├── sim/
│   ├── runner.py         #   MuJoCo クローズドループ push 実行
│   └── keyframe.py       #   6-DOF tool-down IK で 'ready' keyframe 生成
├── analytical/push_com_sim.py   # Stage-0 解析 (limit-surface) シミュレータ
└── viz/                  # grid_video (2x2 動画) + plots (result.png)
scenes/                   # scene/robot XML (+ legacy/ に旧 pusher scene)
scripts/                  # CLI エントリ (tyro) + setup_assets.sh
experiments/              # Stage 0/1+ の解析実験 (legacy, 各自の scene を使用)
tests/                    # pytest
notes/                    # TODO / PLAN / ISSUES / LOGS
```

新しいコントローラは `pusher_slider/controllers/` に追加し
`@register_controller("name")` で登録すれば `make_controller` から使える
(Hogan MPC 本体は不変)。

## 開発

```bash
pixi run python -m pytest tests/ -v     # テスト
pixi run ruff check pusher_slider scripts tests
pixi run ruff format pusher_slider scripts tests
```

依存を変えたら `pixi install` で `pixi.lock` を更新し, `pixi.toml` と一緒にコミットする。
