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

UID/GID は初回起動時に `docker/init-env.sh` が `id -u`/`id -g` をもとに `docker/.env` を自動生成する。

- **VS Code (devcontainer)**: 自動。`devcontainer.json` の `initializeCommand` がスクリプトを呼ぶ
- **CLI (standalone)**: 起動前に1度だけ手動実行
  ```bash
  bash docker/init-env.sh
  ```

CUDA バージョンや DISPLAY 等を上書きしたい場合は、生成された `docker/.env` に追記する (項目は `docker/.env.example` を参照)。`docker/.env` は git 管理外なので、UID/GID をリセットしたい場合はファイルを削除すれば次回起動時に再生成される。

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
| `~/.claude/CLAUDE.md` | `~/.claude/CLAUDE.md` | Claude Code: ユーザーレベル指示 |
| `~/.claude/skills` | `~/.claude/skills` | Claude Code: カスタム skill |
| `~/.claude/agents` | `~/.claude/agents` | Claude Code: サブエージェント定義 |
| `~/.claude/hooks` | `~/.claude/hooks` | Claude Code: hook スクリプト |
| `~/.claude/settings.json` | `~/.claude/settings.json` | Claude Code: global 設定 |
| `~/.claude/keybindings.json` | `~/.claude/keybindings.json` | Claude Code: キーバインド |
| `~/.claude/rules` | `~/.claude/rules` | Claude Code: CLAUDE.md から参照される個人ルール |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | GUI 転送 |
| named volume | `~/.cache/pip` | pip キャッシュ永続化 |

> Claude Code の `projects/` (会話履歴), `todos/`, `.credentials.json` (認証), `~/.claude.json` (MCP) はコンテナ独立。コンテナで初回利用時は `claude` で再ログインが必要。

### VS Code 拡張 (14個)

Claude Code, Python, Pylance, Ruff, Jupyter, Docker, GitLens, Git Graph, Debugpy, YAML, TOML, Markdown, Error Lens, Todo Tree, Spell Checker, Path Intellisense

### entrypoint.sh の動作

1. 初回起動時に zsh の設定ファイルを生成
2. `~/.cache`, `~/.local`, `~/.config`, `~/.claude` を作成
3. `pyproject.toml` があればプロジェクトを editable install
4. `gosu` で非 root ユーザーに切替してコマンドを実行

### Doppler によるシークレット管理 (オプション)

API キー (`WANDB_API_KEY`, `HF_TOKEN`, `OPENAI_API_KEY` 等) を [Doppler](https://www.doppler.com/) 経由でコンテナに注入できる。Doppler CLI はイメージに同梱され、シェル起動時に `/etc/zsh/zshenv` が `doppler secrets download` を実行して全 secret を環境変数として展開する。`DOPPLER_TOKEN` 未設定ならその処理はスキップ (no-op) するので、Doppler を使わない人にはテンプレートが透過的。

#### Doppler 側の作業 (workplace ごとに 1 回)

ブラウザのみで完結:

1. [Doppler dashboard](https://dashboard.doppler.com/) で Project を作成 (例: `research-keys`)
2. デフォルトの 3 environment (`dev` / `stg` / `prd`) のうち `dev` を使う
3. `dev` config を開いて **Add Secret** で必要なキーを登録 (例)

   ```text
   WANDB_API_KEY = <your-wandb-key>
   HF_TOKEN = <your-hf-token>
   ANTHROPIC_API_KEY = <your-anthropic-key>
   ```

4. Project → 該当 config (`dev`) → **Access** タブ → **Service Tokens** → **Generate**
   - Access: **Read** (コンテナからは読み取りのみ)
   - 表示された `dp.st.dev.xxxxxxxxxxxxxxx` をコピー (この画面でしか見られない)

#### マシンごとの作業 (各ホストで 1 回)

ホストにファイル 1 個作るだけ。Doppler CLI のインストール不要:

```bash
mkdir -p ~/.config
echo 'DOPPLER_TOKEN=dp.st.dev.ここにペースト' > ~/.config/doppler.env
chmod 600 ~/.config/doppler.env
```

このファイルは [docker/docker-compose.yaml](docker/docker-compose.yaml) の `env_file:` で読まれ、ホストシェルには load されない。dotfiles repo で `~/.config/<file>` を個別 symlink している運用なら、新規作成する `doppler.env` は symlink されない = tracked にならない。念のため dotfiles repo の `.gitignore` に `doppler.env` を追加しておくと事故防止になる。

#### 起動と確認

```bash
# rebuild が必要 (Doppler CLI を image に同梱するため)
docker compose -f docker/docker-compose.yaml down
docker compose -f docker/docker-compose.yaml build
docker compose -f docker/docker-compose.yaml up -d

# 確認 (コンテナ内シェルで)
docker compose -f docker/docker-compose.yaml exec dev sh -c 'echo $WANDB_API_KEY'
```

#### 運用ポイント

- **auto-discovery**: dashboard で secret を追加するだけで、次回シェル起動時に自動的にコンテナの env に流入する。`docker-compose.yaml` 編集や container rebuild は不要 (新しいシェルを開けば反映)
- **常に最新**: シェル起動ごとに fetch するので、dashboard で値を変更しても次のシェルで反映 (約 500ms の起動時オーバーヘッド)
- **Read-only**: service token は read-only スコープなのでコンテナ側から secret を書き換え不可
- **Token rotation**: 漏洩疑いがあれば dashboard で revoke → 新規生成 → 各マシンの `~/.config/doppler.env` を更新
- **プロジェクトごとに別 config を使いたい場合**: `docker/docker-compose.override.yaml` (gitignored) で `env_file:` を上書きする
- **wandb など `.netrc` ベースのツール**: env var (`WANDB_API_KEY` 等) が優先されるので、Doppler 経由で渡せば `.netrc` マウントは不要にできる

## カスタマイズ例

### GPU 不要の場合

`docker-compose.yaml` の `deploy` セクションと GPU 関連の `environment` を削除し、ベースイメージを `ubuntu:24.04` に変更する。

### 追加サービスが必要な場合

`docker-compose.yaml` に `services` を追加する (例: DB, Redis, Ollama 等)。
