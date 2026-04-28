# Dev Container のパーミッション/UID 既知問題

ホスト ↔ コンテナ間の UID 不整合と、それに付随する周辺ツール (git, sudo) の挙動について。
今後同じ症状を踏んだ人がループせず復旧できるようにメモ。

## 症状

このリポジトリの dev container を VS Code で開いて作業していると、以下が発生し得る:

1. Claude Code のハーネスが `EACCES: permission denied, mkdir '/home/<user>/.claude/session-env/<UUID>'` で停止する
2. `git fetch` / `git checkout` が `fatal: detected dubious ownership in repository at '/workspace'` で停止する
3. `git config --global --add safe.directory ...` が `error: could not write config file ...: Device or resource busy` で書けない
4. `git fetch` 自体は通っても `error: cannot open '.git/FETCH_HEAD': Permission denied` で書き込みに失敗する
5. `sudo` が NOPASSWD 設定済みのはずなのにパスワードを要求してきて、結局通らない

## 根本原因

**コンテナ内ユーザの UID とホスト側ユーザの UID が一致していない。**

- ホスト: `atsushi.kuno` の UID は 5008
- コンテナ: `atsushi.kuno` の UID は 1000 (`docker/docker-compose.yaml` の `HOST_UID: ${HOST_UID:-1000}` でビルド時に `HOST_UID` 未設定 → デフォルト 1000)

`/workspace` も `~/.claude` も bind mount で、bind mount は UID を翻訳せずホスト側の UID 番号をそのまま見せる。結果としてホスト (または別の UID で動く別コンテナ) が書き込んだファイルは、このコンテナ内では UID 5008 所有として見え、UID 1000 の自分からは書けない。

これが土台で、症状 1, 2, 4 はすべてここから派生する。

### 派生問題

#### Git "dubious ownership" (症状 2, 3)

CVE-2022-24765 対策として git 2.35.2 以降は、リポジトリ (`.git`) の所有者が現在のユーザと異なる場合に処理を停止する。

> "Git v2.35.2 introduced behavior that stop[s] when its directory traversal changes ownership from the current user."
> — [GitHub Blog: CVE-2022-24765](https://github.blog/open-source/git/git-security-vulnerability-announced/)

回避策として `safe.directory` 設定が用意されているが、`docker/docker-compose.yaml` でホストの `~/.gitconfig` を `:ro` でマウントしているため `git config --global` で書き込めない (症状 3)。インラインで `git -c safe.directory=/workspace ...` を渡せば fetch 自体は通るが、その先で `.git/FETCH_HEAD` への書き込みが UID 不整合で蹴られる (症状 4) ので結局根本対処にはならない。

#### sudo NOPASSWD が効かない (症状 5)

`docker/Dockerfile` の sudoers 設定:

```dockerfile
echo "${HOST_USER} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${HOST_USER}
```

`HOST_USER=atsushi.kuno` の場合、出力先は `/etc/sudoers.d/atsushi.kuno` だが、sudo の `#includedir` は仕様上ドットを含むファイル名を黙ってスキップする:

> "sudo will suspend processing of the current file and read each file in /etc/sudoers.d, **skipping file names that end in '~' or contain a '.' character** to avoid causing problems with package manager or editor temporary/backup files."
> — [sudoers(5) manual](https://man.archlinux.org/man/sudoers.5)

ホスト側ユーザ名がドット入り (`firstname.lastname` 規則) のチームでは、このコンテナの sudo は最初から効いていない。ユーザ自身のパスワードも未設定なので、sudo は実質一切使えない。

## 当面の回避策

UID 不整合の影響を受けたファイルを root 権限で chown しなおす。コンテナ内 sudo が壊れているので、**ホスト側から `docker exec -u 0`** で実行する。

### Claude Code の session-env が壊れた場合

```bash
# ホスト側
docker exec -u 0 wisp-container-blackwell-wisp-dev-blackwell-1 \
    chown -R <container-user>:<container-user> /home/<container-user>/.claude/session-env
```

### git 操作

コンテナ内で git すると上記 1〜4 を踏むので、**ホスト側で git する** のが最も確実。`/workspace` は bind mount なのでホスト側で checkout した結果はそのままコンテナ内に反映される (VS Code のソース管理パネルが古いブランチ名を表示する場合は "Developer: Reload Window" で更新)。

```bash
# ホスト側、プロジェクトディレクトリで
git fetch origin <branch>
git checkout <branch>
```

どうしてもコンテナ内で git したい場合は `docker exec -u 0 -w /workspace <container> git -c safe.directory=/workspace ...` を経由する。

## 恒久対応 (リビルド時に当てる)

### 1. `HOST_UID` をホストに合わせる

ビルド時に環境変数を渡す:

```bash
cd docker
HOST_UID=$(id -u) HOST_GID=$(id -g) docker compose build
docker compose up -d --force-recreate
```

`docker/.env` に書いておけば毎回指定不要:

```
HOST_UID=5008
HOST_GID=5008
```

これでコンテナ内ユーザとホスト側ユーザの UID/GID が一致し、bind mount 越しのファイル所有権ズレが解消する。git の dubious ownership も発生しなくなり、`safe.directory` も不要になる。

### 2. sudoers ファイル名からドットを除く

`docker/Dockerfile` を以下のように修正:

```diff
-    && echo "${HOST_USER} ALL=(ALL) NOPASSWD:ALL" \
-       >> /etc/sudoers.d/${HOST_USER} \
-    && chmod 0440 /etc/sudoers.d/${HOST_USER}
+    && echo "${HOST_USER} ALL=(ALL) NOPASSWD:ALL" \
+       > /etc/sudoers.d/99-devuser \
+    && chmod 0440 /etc/sudoers.d/99-devuser
```

(`>>` を `>` に変更しているのは、ドット入り旧ファイルの再生成で混乱しないようにするため。)

## この問題は Claude 固有ではない

UID 不整合はバインドマウントを使う dev container 全般の古典的な問題で、影響範囲は広い:

- pip / npm / cargo の cache や lockfile 書き込み
- pyright / rust-analyzer / clangd 等 LSP の cache
- Jupyter ノートブックの保存
- ログ・成果物を `/workspace` 配下に吐く全 CLI

Claude Code の session-env は新規 UUID ディレクトリを毎セッション作成するため踏み抜きやすかっただけで、症状の出方が違うだけで原因は同じ。

`safe.directory` の問題は git 固有、sudoers のドットの問題は sudo + ドット入りユーザ名の組み合わせ固有 (それぞれ独立) で、これらは UID 問題とは別軸。

## 参考リンク

- [sudoers(5) — #includedir のファイル名規則](https://man.archlinux.org/man/sudoers.5)
- [GitHub Security Blog — CVE-2022-24765 / safe.directory](https://github.blog/open-source/git/git-security-vulnerability-announced/)
- [Docker docs — Bind mounts (UID は翻訳されない)](https://docs.docker.com/storage/bind-mounts/)
