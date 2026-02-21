# Devcontainer Base Template

NVIDIA CUDA + Ubuntu 24.04 ベースの VS Code devcontainer テンプレート。
GPU 開発環境を新規プロジェクトごとに素早く立ち上げるためのベース設定。

## ディレクトリ構成

```
template/
├── .devcontainer/
│   └── devcontainer.json       # VS Code 拡張・Python・シェル設定
├── docker/
│   ├── Dockerfile              # nvidia/cuda + Ubuntu 24.04, Python venv, 非 root ユーザー
│   ├── docker-compose.yaml     # GPU・ボリューム・ipc: host
│   ├── entrypoint.sh           # zsh 初期化 + gosu による非 root 切替
│   ├── requirements.txt        # 共通 dev ツール (ruff, pytest, debugpy 等)
│   └── .env.example            # マシン固有の設定テンプレート
├── .dockerignore               # ホワイトリスト方式のビルドコンテキスト制御
├── .gitignore                  # docker/.env 等を除外
└── README.md
```

## 使い方

### 1. テンプレートをコピー

```bash
cp -r template/ my-new-project/
cd my-new-project/
```

### 2. プロジェクト名を書き換える

各ファイル内の `TODO` コメントを検索し、プロジェクトに合わせて変更する。

| ファイル | 変更箇所 |
|---------|---------|
| `docker/docker-compose.yaml` | `name`, サービス名 (`dev`), `image` |
| `.devcontainer/devcontainer.json` | `name`, `service` |

### 3. 環境変数を設定

```bash
cp docker/.env.example docker/.env
```

`docker/.env` を編集して CUDA バージョンや UID/GID を自分のマシンに合わせる。

### 4. プロジェクト固有の依存を追加

`docker/requirements.txt` にプロジェクトで使うパッケージを追記する。

### 5. コンテナを起動

**VS Code (devcontainer)**:

コマンドパレット → `Dev Containers: Reopen in Container`

**CLI (standalone)**:

```bash
cd docker
docker compose build
docker compose up -d
docker compose exec dev zsh
```

## 含まれる設定

### ベースイメージ

`nvidia/cuda:${CUDA_VERSION}-devel-ubuntu24.04` — CUDA バージョンは `.env` の `CUDA_VERSION` で切替可能。

### GPU サポート

デフォルトで NVIDIA GPU 全台を割当。`ipc: host` により PyTorch DataLoader / NCCL の共有メモリも有効。

### 非 root ユーザー

ホストの UID/GID をビルド時に注入し、コンテナ内でもホストと同じ権限で動作。`gosu` でランタイム切替。

### ボリュームマウント

| ホスト | コンテナ | 用途 |
|-------|---------|------|
| プロジェクトルート | `/workspace` | ワークスペース |
| `~/.ssh` | `~/.ssh` (ro) | SSH 鍵 |
| `~/.gitconfig` | `~/.gitconfig` (ro) | Git 設定 |
| `~/.netrc` | `~/.netrc` (ro) | wandb 等の認証 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | GUI 転送 |
| named volume | `~/.cache/pip` | pip キャッシュ永続化 |

### VS Code 拡張 (14個)

Claude Code, Python, Pylance, Ruff, Jupyter, Docker, GitLens, Git Graph, Debugpy, YAML, TOML, Markdown, Error Lens, Todo Tree, Spell Checker, Path Intellisense

### entrypoint.sh の動作

1. 初回起動時に zsh の設定ファイルを生成
2. `~/.cache`, `~/.local`, `~/.config`, `~/.claude` を作成
3. `pyproject.toml` があればプロジェクトを editable install
4. `gosu` で非 root ユーザーに切替してコマンドを実行

## カスタマイズ例

### GPU 不要の場合

`docker-compose.yaml` の `deploy` セクションと GPU 関連の `environment` を削除し、ベースイメージを `ubuntu:24.04` に変更する。

### 追加サービスが必要な場合

`docker-compose.yaml` に `services` を追加する (例: DB, Redis, Ollama 等)。
