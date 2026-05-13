# =============================================================================
# Development Container — Base Template
# =============================================================================
# Base: NVIDIA CUDA + Ubuntu 24.04
# Adds:  pixi (Python env manager), GPU rendering libs, dev tooling, non-root user
#
# Build:  docker compose build
# Run:    docker compose up -d
# Attach: docker compose exec dev zsh
# =============================================================================

ARG CUDA_VERSION=12.8.0
ARG UBUNTU_VERSION=24.04
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu${UBUNTU_VERSION}

ENV DEBIAN_FRONTEND=noninteractive

# ---- Build-time configurables (defaults align with .env.example) ------------
ARG LOCALE=en_US.UTF-8
ARG WORKSPACE_DIR=/workspace

# ---- System packages --------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        # Shell & terminal
        zsh \
        tmux \
        # VCS & build essentials
        git \
        git-lfs \
        curl \
        wget \
        unzip \
        build-essential \
        cmake \
        # GPU rendering (EGL / OSMesa for headless)
        libegl1 \
        libgl1 \
        libgles2 \
        libosmesa6-dev \
        libglfw3 \
        libglfw3-dev \
        # Editor (fallback)
        nano \
        vim \
        # Utilities
        sudo \
        gosu \
        locales \
        ca-certificates \
        openssh-client \
        less \
        jq \
    && locale-gen ${LOCALE} \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=${LOCALE}
ENV LC_ALL=${LOCALE}

# ---- pixi (Python environment manager) --------------------------------------
# The binary lands in /usr/local/bin so all users see it on PATH without extra
# shell config. PIXI_HOME (where pixi keeps global envs / config) is set later,
# after the non-root user is created, so writes go to that user's home and not
# to a root-owned system directory.
# The actual project environment is materialised at runtime (entrypoint) from
# ${WORKSPACE_DIR}/pixi.toml into ${WORKSPACE_DIR}/.pixi/envs/default.
ENV PIXI_BIN_DIR=/usr/local/bin PIXI_NO_PATH_UPDATE=1
RUN curl -fsSL https://pixi.sh/install.sh | sh \
    && pixi --version

# ---- Doppler CLI (fetches secrets per-shell via /etc/zsh/zshenv below) -----
RUN curl -Ls --tlsv1.2 --proto "=https" --retry 3 https://cli.doppler.com/install.sh \
        | sh -s -- --no-package-manager \
    && doppler --version

# /etc/zsh/zshenv is sourced by every zsh invocation (login, non-login,
# interactive, non-interactive — including `docker compose exec dev zsh`).
# This is the only reliable way to get secrets into exec'd shells, since
# entrypoint-set env vars do not propagate through `docker exec`.
RUN mkdir -p /etc/zsh && cat >> /etc/zsh/zshenv <<'EOF'

# gosu/docker-exec do not propagate $USER (only $HOME). Restore it from
# /etc/passwd so $USER works uniformly across all zsh entry points.
export USER="${USER:-$(id -un)}"
export LOGNAME="${LOGNAME:-$USER}"

# Doppler: fetch secrets when DOPPLER_TOKEN is set in the container env.
# Silently no-op when token is missing or fetch fails.
if [ -n "${DOPPLER_TOKEN:-}" ] && command -v doppler >/dev/null 2>&1; then
    if _doppler_env="$(doppler secrets download --no-file --format=env --silent 2>/dev/null)"; then
        eval "$_doppler_env"
    fi
    unset _doppler_env DOPPLER_TOKEN
fi

# Expose the `claude` CLI bundled with the VS Code Claude Code extension.
# The extension dir carries a version suffix that changes on updates, and
# .vscode-server is only populated after VS Code attaches, so we resolve it
# via a zsh glob at shell startup. Lives here (not in the generated ~/.zshrc)
# so it is part of the image and updates with a rebuild — ~/.zshrc is only
# written once by entrypoint.sh and never refreshed afterwards.
# Glob qualifiers: (N) null-glob (no error when absent), (.) regular files,
# (x) executable, (oc) sort by ctime descending so the newest install wins,
# [1] take the first. No-ops silently when the extension isn't present.
_claude_bin=("${HOME}"/.vscode-server/extensions/anthropic.claude-code-*/resources/native-binary/claude(N.xoc[1]))
if (( ${#_claude_bin} )); then
    export PATH="${_claude_bin[1]:h}:${PATH}"
fi
unset _claude_bin
EOF

# ---- Non-root user (UID/GID injected at build time) ------------------------
ARG HOST_UID=1000
ARG HOST_GID=1000
ARG HOST_USER=developer

RUN if getent group ${HOST_GID} >/dev/null 2>&1; then \
        existing_grp=$(getent group ${HOST_GID} | cut -d: -f1); \
        [ "$existing_grp" != "${HOST_USER}" ] && groupmod -n ${HOST_USER} "$existing_grp"; \
    else \
        groupadd -g ${HOST_GID} ${HOST_USER}; \
    fi \
    && if getent passwd ${HOST_UID} >/dev/null 2>&1; then \
         existing_usr=$(getent passwd ${HOST_UID} | cut -d: -f1); \
         usermod -l ${HOST_USER} -s /bin/zsh -g ${HOST_GID} \
                 -d /home/${HOST_USER} -m "$existing_usr"; \
       else \
         useradd -m -s /bin/zsh -u ${HOST_UID} -g ${HOST_GID} ${HOST_USER}; \
       fi \
    && echo "${HOST_USER} ALL=(ALL) NOPASSWD:ALL" \
       > /etc/sudoers.d/container-user \
    && chmod 0440 /etc/sudoers.d/container-user

# Point pixi at the non-root user's home for global envs / config. Without
# this, `pixi info` and other global ops try to create /usr/local/envs and
# fail with Permission denied. The project-local env in ${WORKSPACE_DIR}/.pixi
# is unaffected. entrypoint.sh chowns this directory at startup.
ENV PIXI_HOME=/home/${HOST_USER}/.pixi

# ---- Entrypoint -------------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

# Persist WORKSPACE_DIR as an env var so entrypoint.sh and shells can reuse it.
ENV WORKSPACE_DIR=${WORKSPACE_DIR}
WORKDIR ${WORKSPACE_DIR}
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]
