#!/bin/bash
set -e

# =============================================================================
# Container entrypoint
# - Initialises shell config for the runtime user
# - Materialises the pixi environment from ${WORKSPACE_DIR}/pixi.toml
# - Installs project in editable mode (if pyproject.toml exists)
# - Drops to non-root user via gosu
# =============================================================================

TARGET_USER="${HOST_USER:-developer}"
TARGET_HOME=$(eval echo "~${TARGET_USER}" 2>/dev/null || echo "/home/${TARGET_USER}")
TARGET_GROUP=$(id -gn "${TARGET_USER}" 2>/dev/null || echo "${TARGET_USER}")
WORKSPACE_DIR="${WORKSPACE_DIR:-/workspace}"
PIXI_ENV_BIN="${WORKSPACE_DIR}/.pixi/envs/default/bin"

# ---- zsh bootstrap (only on first run) --------------------------------------
if [ ! -f "${TARGET_HOME}/.zshrc" ]; then
    mkdir -p "${TARGET_HOME}"
    cat > "${TARGET_HOME}/.zshrc" << 'ZSHRC'
# Minimal zsh config
autoload -Uz compinit && compinit
autoload -Uz vcs_info
precmd() { vcs_info }
zstyle ':vcs_info:git:*' formats ' (%b)'

setopt PROMPT_SUBST
PROMPT='%F{cyan}%~%f${vcs_info_msg_0_} %F{green}>%f '

# History
HISTFILE=~/.zsh_history
HISTSIZE=10000
SAVEHIST=10000
setopt SHARE_HISTORY HIST_IGNORE_DUPS

# Aliases
alias ll='ls -lah --color=auto'
alias la='ls -A --color=auto'

# Pixi-managed Python environment activation for interactive shells.
# Uses `pixi shell-hook` so that any [activation] / [activation.env] entries
# in pixi.toml (e.g. project-specific env vars) are applied alongside PATH.
# Falls back silently when the env hasn't been materialised yet (first boot,
# or before `pixi install` completes). $WORKSPACE_DIR is set by the Dockerfile.
if [ -d "${WORKSPACE_DIR}/.pixi/envs/default" ] && command -v pixi >/dev/null 2>&1; then
    eval "$(pixi shell-hook --manifest-path "${WORKSPACE_DIR}" 2>/dev/null)"
fi

# Note: the `claude` CLI from the VS Code Claude Code extension is put on PATH
# by /etc/zsh/zshenv (installed by the Dockerfile), not here — that file is
# part of the image and refreshes on rebuild, whereas this ~/.zshrc is written
# once and never updated.
ZSHRC
    chown "${TARGET_USER}:${TARGET_GROUP}" "${TARGET_HOME}/.zshrc"
fi

# ---- Ensure user directories exist with correct ownership --------------------
# ~/.cache/rattler is mounted as a named volume (see docker-compose.yaml) and
# Docker creates it root-owned; chown the mount point so the non-root user can
# write into it. We only chown the mount point itself, not the contents, to
# avoid trampling existing cache state on subsequent boots.
# ~/.pixi is set as PIXI_HOME in the Dockerfile; pixi writes its global envs
# and config there, so it must be writable by the non-root user.
for d in \
    "${TARGET_HOME}/.cache" \
    "${TARGET_HOME}/.cache/rattler" \
    "${TARGET_HOME}/.local" \
    "${TARGET_HOME}/.config" \
    "${TARGET_HOME}/.claude" \
    "${TARGET_HOME}/.pixi"; do
    mkdir -p "$d"
    chown "${TARGET_USER}:${TARGET_GROUP}" "$d" 2>/dev/null || true
done

# ---- Materialise pixi environment from ${WORKSPACE_DIR}/pixi.toml -----------
# Runs as the target (non-root) user so that .pixi/ is owned by them.
#
# Strategy:
#   - First run (no pixi.lock): plain `pixi install` to generate the lock.
#   - Subsequent runs:           `pixi install --locked` to verify lock matches
#                                pixi.toml and fail loudly on drift, instead of
#                                silently relocking and producing a different
#                                env from teammates'.
# After editing pixi.toml, run `pixi install` (no flag) inside the container
# once to refresh pixi.lock, then commit both files together.
if [ -f "${WORKSPACE_DIR}/pixi.toml" ] || [ -f "${WORKSPACE_DIR}/pyproject.toml" ]; then
    if [ -f "${WORKSPACE_DIR}/pixi.lock" ]; then
        PIXI_INSTALL_FLAGS="--locked"
        echo "Verifying pixi environment against lockfile..."
    else
        PIXI_INSTALL_FLAGS=""
        echo "Resolving pixi environment (no lockfile yet)..."
    fi
    if [ "$(id -u)" = "0" ] && [ "${TARGET_USER}" != "root" ]; then
        gosu "${TARGET_USER}" pixi install ${PIXI_INSTALL_FLAGS} --manifest-path "${WORKSPACE_DIR}" 2>&1 | tail -5 || \
            echo "WARNING: pixi install failed (non-fatal, continuing...)"
    else
        pixi install ${PIXI_INSTALL_FLAGS} --manifest-path "${WORKSPACE_DIR}" 2>&1 | tail -5 || \
            echo "WARNING: pixi install failed (non-fatal, continuing...)"
    fi
fi

# ---- Install project in editable mode (if pyproject.toml exists) ------------
# Uses the pixi env's pip so the project lands in ${WORKSPACE_DIR}/.pixi/envs/default.
if [ -f "${WORKSPACE_DIR}/pyproject.toml" ] && [ -x "${PIXI_ENV_BIN}/pip" ]; then
    echo "Installing project in editable mode..."
    if [ "$(id -u)" = "0" ] && [ "${TARGET_USER}" != "root" ]; then
        gosu "${TARGET_USER}" "${PIXI_ENV_BIN}/pip" install --no-deps -e "${WORKSPACE_DIR}" 2>&1 | tail -1 || \
            echo "WARNING: editable install failed (non-fatal, continuing...)"
    else
        "${PIXI_ENV_BIN}/pip" install --no-deps -e "${WORKSPACE_DIR}" 2>&1 | tail -1 || \
            echo "WARNING: editable install failed (non-fatal, continuing...)"
    fi
fi

# ---- Drop to non-root user and exec command ---------------------------------
# Doppler secrets are fetched per-shell via /etc/zsh/zshenv (installed by the
# Dockerfile). That ensures `docker compose exec` shells also receive secrets,
# which env vars exported here would not reach.
if [ "$(id -u)" = "0" ] && [ "${TARGET_USER}" != "root" ]; then
    exec gosu "${TARGET_USER}" "$@"
else
    exec "$@"
fi
