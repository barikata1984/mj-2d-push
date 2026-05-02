# =============================================================================
# Development Container — Base Template
# =============================================================================
# Base: NVIDIA CUDA + Ubuntu 24.04
# Adds:  Python venv, GPU rendering libs, dev tooling, non-root user
#
# Build:  docker compose build
# Run:    docker compose up -d
# Attach: docker compose exec dev zsh
# =============================================================================

ARG CUDA_VERSION=12.8.0
FROM nvidia/cuda:${CUDA_VERSION}-devel-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive

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
        # Python 3
        python3 \
        python3-pip \
        python3-venv \
        python3-dev \
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
    && locale-gen en_US.UTF-8 \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---- Python virtual environment ---------------------------------------------
ENV VIRTUAL_ENV=/opt/venv
RUN python3 -m venv ${VIRTUAL_ENV}
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

# ---- Pip packages (from separate requirements file) -------------------------
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --upgrade pip \
    && pip install --no-cache-dir -r /tmp/requirements.txt \
    && rm /tmp/requirements.txt

# ---- Doppler CLI (fetches secrets per-shell via /etc/zsh/zshenv below) -----
RUN curl -Ls --tlsv1.2 --proto "=https" --retry 3 https://cli.doppler.com/install.sh \
        | sh -s -- --no-package-manager \
    && doppler --version

# /etc/zsh/zshenv is sourced by every zsh invocation (login, non-login,
# interactive, non-interactive — including `docker compose exec dev zsh`).
# This is the only reliable way to get secrets into exec'd shells, since
# entrypoint-set env vars do not propagate through `docker exec`.
RUN mkdir -p /etc/zsh && cat >> /etc/zsh/zshenv <<'EOF'

# Doppler: fetch secrets when DOPPLER_TOKEN is set in the container env.
# Silently no-op when token is missing or fetch fails.
if [ -n "${DOPPLER_TOKEN:-}" ] && command -v doppler >/dev/null 2>&1; then
    if _doppler_env="$(doppler secrets download --no-file --format=env --silent 2>/dev/null)"; then
        eval "$_doppler_env"
    fi
    unset _doppler_env DOPPLER_TOKEN
fi
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

# ---- Entrypoint -------------------------------------------------------------
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["sleep", "infinity"]
