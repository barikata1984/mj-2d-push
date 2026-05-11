# Devcontainer Base Template

NVIDIA CUDA + Ubuntu (既定 24.04) ベースの VS Code devcontainer テンプレート。Python 環境は [pixi](https://pixi.sh/) で管理し、`pixi.lock` でビット単位の再現性を担保する。
GPU 開発環境を新規プロジェクトごとに素早く立ち上げるためのベース設定。

## ディレクトリ構成

```
<repo-root>/
├── .devcontainer/
│   ├── devcontainer.json       # VS Code 拡張・Python・シェル設定
│   ├── Dockerfile              # nvidia/cuda + Ubuntu (既定 24.04), pixi, 非 root ユーザー
│   ├── docker-compose.yaml     # GPU・ボリューム・ipc: host
│   ├── entrypoint.sh           # zsh 初期化 + pixi install + gosu による非 root 切替
│   ├── init-env.sh             # pixi.toml から .env 生成 (SSoT 派生)
│   ├── .dockerignore           # ビルドコンテキスト除外 (.env / .pixi 等)
│   └── .env.example            # マシン固有の設定テンプレート
├── pixi.toml                   # プロジェクト名・Python バージョン・依存パッケージ定義 (SSoT)
├── pixi.lock                   # 解決済み依存の凍結 (pixi が自動生成)
├── .gitignore                  # .devcontainer/.env / .pixi/ 等を除外
└── README.md
```

## 使い方

### 1. リポジトリをクローン

```bash
git clone <this-repo-url> my-new-project
cd my-new-project
```

### 2. プロジェクト名を書き換える

[pixi.toml](pixi.toml) の `[workspace].name` を新しい名前に書き換える。これが image tag・compose project name・Claude Code 履歴ディレクトリ名を貫く **唯一の SSoT (Single Source of Truth)**。

```toml
[workspace]
name = "your-new-project-name"   # ここ 1 箇所だけ書き換える
```

`init-env.sh` がこの値を `tomllib` で抽出し、`.devcontainer/.env` の `COMPOSE_PROJECT_NAME` に流し込む。`docker-compose.yaml` / `Dockerfile` / `entrypoint.sh` は `${COMPOSE_PROJECT_NAME}` 補間で参照しているため触る必要なし。古い pixi.toml で `[project]` を使っているリポジトリも互換のため引き続き読める。

ただし [.devcontainer/devcontainer.json](.devcontainer/devcontainer.json) の `"name"` フィールド（VS Code 左下に "Dev Container: ..." と表示されるラベル）は devcontainer 仕様上 `.env` 補間に対応しないため SSoT から自動派生しない。プロジェクト識別を視覚的に揃えたい場合は、ここも併せて手動更新する：

```jsonc
"name": "Your Project Name",  // 表示ラベル。Docker 識別子 (image/container 名) には影響しない
```

更新後は `Cmd/Ctrl+Shift+P → Developer: Reload Window` で反映（リビルド不要）。

### 3. 環境変数を設定

[.devcontainer/.env](.devcontainer/.env) は `.devcontainer/init-env.sh` が以下を素材に自動生成する：

- pixi.toml の `[workspace].name` → `COMPOSE_PROJECT_NAME`
- ホストの UID/GID → `HOST_UID` / `HOST_GID`
- 環境変数または既定値 → `CUDA_VERSION` / `UBUNTU_VERSION` / `WORKSPACE_DIR` / `DEFAULT_USER` / `LOCALE` / `DISPLAY_NUM` / `NVIDIA_VISIBLE_DEVICES`

実行タイミング:
- **VS Code (devcontainer)**: 自動。`devcontainer.json` の `initializeCommand` がスクリプトを呼ぶ
- **CLI (standalone)**: 起動前に 1 度だけ手動実行
  ```bash
  bash .devcontainer/init-env.sh
  ```

既定値を上書きしたい場合の選択肢:
- 生成された `.devcontainer/.env` を直接編集する（最も手軽）
- `.devcontainer/.env` を削除し、override をシェルで export してから再実行：
  ```bash
  rm .devcontainer/.env
  CUDA_VERSION=12.6.0 UBUNTU_VERSION=22.04 LOCALE=ja_JP.UTF-8 bash .devcontainer/init-env.sh
  ```

各変数の意味は [.devcontainer/.env.example](.devcontainer/.env.example) を参照。`.devcontainer/.env` は git 管理外。

### 4. プロジェクト固有の依存を追加

ルート直下の [pixi.toml](pixi.toml) に追記する。conda-forge にあるパッケージは `[dependencies]`、PyPI 専用パッケージは `[pypi-dependencies]` に書く：

```toml
[dependencies]
python = "3.12.*"
numpy = "*"
pytorch = "*"

[pypi-dependencies]
some-pypi-only-package = "*"
```

編集後は **コンテナ内で** `pixi install` を実行して [pixi.lock](pixi.lock) を更新し、`pixi.toml` と `pixi.lock` を一緒にコミットする。コンテナ起動時には `entrypoint.sh` が `pixi install --locked` で lock 整合を検証するため、lock 更新を忘れると起動時に警告が出る。

### 5. コンテナを起動

**VS Code (devcontainer)**:

コマンドパレット → `Dev Containers: Reopen in Container`

**CLI (standalone)**: ホスト側に pixi が入っていれば、リポジトリルートから 1 コマンドで全操作可能（[pixi.toml](pixi.toml) の `[tasks]` を参照）：

```bash
pixi run build       # docker compose build
pixi run up          # docker compose up -d
pixi run shell       # docker compose exec dev zsh
pixi run logs        # docker compose logs -f
pixi run down        # docker compose down
```

pixi を使わない場合は従来どおり：

```bash
cd .devcontainer
docker compose build
docker compose up -d
docker compose exec dev zsh
```

## 含まれる設定

### ベースイメージ

`nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}` — CUDA・Ubuntu のバージョンは `.env` の `CUDA_VERSION` / `UBUNTU_VERSION` で切替可能。

### GPU サポート

デフォルトで NVIDIA GPU 全台を割当。`ipc: host` により PyTorch DataLoader / NCCL の共有メモリも有効。

### 非 root ユーザー

ホストの UID/GID をビルド時に注入し、コンテナ内でもホストと同じ権限で動作。`gosu` でランタイム切替。

### ボリュームマウント

| ホスト | コンテナ | 用途 |
|-------|---------|------|
| プロジェクトルート | `${WORKSPACE_DIR}` (既定 `/workspace`) | ワークスペース |
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
| named volume | `~/.cache/rattler` | pixi/conda パッケージキャッシュ永続化 |

> Claude Code の会話履歴・todos・shell snapshots は別途ホスト側 `~/.claude-stacks/${COMPOSE_PROJECT_NAME}/` に bind mount され、rebuild を跨いで保持される（詳細は[後述](#claude-code-履歴の永続化)）。一方 `.credentials.json` (認証) と `~/.claude.json` (MCP) はコンテナ独立で、初回利用時は `claude` で再ログインが必要。

### VS Code 拡張 (14個)

Claude Code, Python, Pylance, Ruff, Jupyter, Docker, GitLens, Git Graph, Debugpy, YAML, TOML, Markdown, Error Lens, Todo Tree, Spell Checker, Path Intellisense

### pixi のディレクトリ配置

| パス | 役割 |
|---|---|
| `/usr/local/bin/pixi` | pixi バイナリ本体（システムワイド、全ユーザーから PATH 経由で見える） |
| `~/.pixi/` | `PIXI_HOME`。pixi の global env / config 置き場（非 root ユーザー所有） |
| `${WORKSPACE_DIR}/.pixi/envs/default/` | プロジェクト固有の Python 環境本体（`pixi install` で生成、`.gitignore` 対象） |
| `~/.cache/rattler/` | conda/PyPI パッケージのダウンロードキャッシュ（named volume で永続化） |

### entrypoint.sh の動作

1. 初回起動時に zsh の設定ファイルを生成（`pixi shell-hook` を `.zshrc` に組み込み）
2. `~/.cache`, `~/.cache/rattler`, `~/.local`, `~/.config`, `~/.claude`, `~/.pixi` を作成 & 非 root ユーザー所有に変更（named volume / `PIXI_HOME` の root 所有を補正）
3. `${WORKSPACE_DIR}/pixi.toml` から pixi 環境を `${WORKSPACE_DIR}/.pixi/envs/default` に materialize
   - `pixi.lock` がある場合: `pixi install --locked` で整合検証
   - 無い場合: `pixi install` で lock を新規生成
4. `pyproject.toml` があれば pixi 環境の pip でプロジェクトを editable install
5. `gosu` で非 root ユーザーに切替してコマンドを実行

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

このファイルは [.devcontainer/docker-compose.yaml](.devcontainer/docker-compose.yaml) の `env_file:` で読まれ、ホストシェルには load されない。dotfiles repo で `~/.config/<file>` を個別 symlink している運用なら、新規作成する `doppler.env` は symlink されない = tracked にならない。念のため dotfiles repo の `.gitignore` に `doppler.env` を追加しておくと事故防止になる。

#### 起動と確認

```bash
# rebuild が必要 (Doppler CLI を image に同梱するため)
docker compose -f .devcontainer/docker-compose.yaml down
docker compose -f .devcontainer/docker-compose.yaml build
docker compose -f .devcontainer/docker-compose.yaml up -d

# 確認 (コンテナ内シェルで)
docker compose -f .devcontainer/docker-compose.yaml exec dev sh -c 'echo $WANDB_API_KEY'
```

#### 運用ポイント

- **auto-discovery**: dashboard で secret を追加するだけで、次回シェル起動時に自動的にコンテナの env に流入する。`docker-compose.yaml` 編集や container rebuild は不要 (新しいシェルを開けば反映)
- **常に最新**: シェル起動ごとに fetch するので、dashboard で値を変更しても次のシェルで反映 (約 500ms の起動時オーバーヘッド)
- **Read-only**: service token は read-only スコープなのでコンテナ側から secret を書き換え不可
- **Token rotation**: 漏洩疑いがあれば dashboard で revoke → 新規生成 → 各マシンの `~/.config/doppler.env` を更新
- **プロジェクトごとに別 config を使いたい場合**: `.devcontainer/docker-compose.override.yaml` (gitignored) で `env_file:` を上書きする
- **wandb など `.netrc` ベースのツール**: env var (`WANDB_API_KEY` 等) が優先されるので、Doppler 経由で渡せば `.netrc` マウントは不要にできる

### Claude Code 履歴の永続化

Claude Code のセッション履歴（`~/.claude/{projects,sessions,todos,shell-snapshots}` の 4 ディレクトリ）はホスト側 `~/.claude-stacks/${COMPOSE_PROJECT_NAME}/` 配下に bind mount され、`docker compose down` や rebuild を跨いで保持される。コンテナ内で `claude --resume` すれば過去スレッドを選択できる。

| カテゴリ | 役割 |
|---|---|
| `projects/<encoded-cwd>/<uuid>.jsonl` | 会話本体（resume の読み元） |
| `sessions/<sid>.json` | セッションメタ |
| `todos/` | TodoWrite 状態 |
| `shell-snapshots/` | resume 時に必要なシェル状態 |

- ホスト側ディレクトリは [.devcontainer/init-env.sh](.devcontainer/init-env.sh) が `mkdir -p` で事前作成（ホスト UID 所有を保証）
- `COMPOSE_PROJECT_NAME` 単位で独立。同テンプレートを複数プロジェクトでフォークしても履歴は混ざらない
- 認証 (`.credentials.json`) と telemetry (`statsig/`, `ide/`) はマウント対象外。コンテナごとに `/login` 必要
- マシン跨ぎ同期は対象外（必要なら `borg`/`restic` 等で `~/.claude-stacks/` をバックアップ）

## カスタマイズ例

### GPU 不要の場合

`docker-compose.yaml` の `deploy` セクションと GPU 関連の `environment` を削除し、ベースイメージを `ubuntu:24.04` に変更する。

### 追加サービスが必要な場合

`docker-compose.yaml` に `services` を追加する (例: DB, Redis, Ollama 等)。
