FROM mcr.microsoft.com/devcontainers/python:1-3.12-bookworm

# Dependency versions here must stay aligned with the CI workflows:
#   * Boost: libboost-all-dev (Debian bookworm ships 1.74, within the
#     1.71...1.83 range declared in CMakeLists.txt and matching ubuntu-22.04
#     / ubuntu-24.04 runners).
#   * Eigen3: libeigen3-dev 3.4.x (matches ubuntu runners).
#   * MuJoCo: 3.3.2 — pinned explicitly below via the MUJOCO_VERSION ARG
#     (must match pyproject.toml and CMakeLists.txt find_package(mujoco 3.3.2)).
RUN rm -f /etc/apt/sources.list.d/yarn.list \
    && apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        git \
        libeigen3-dev \
        libboost-all-dev \
        libgl1 \
        libgl1-mesa-dri \
        libglfw3 \
        libosmesa6 \
        libx11-6 \
        libxcursor1 \
        libxinerama1 \
        libxrandr2 \
        libxi6 \
        curl \
        ca-certificates \
        doxygen \
        graphviz \
    && rm -rf /var/lib/apt/lists/*

# MuJoCo C++ release (matches the version pinned in pyproject.toml / CMakeLists.txt).
ARG MUJOCO_VERSION=3.3.2
RUN curl -fsSL "https://github.com/google-deepmind/mujoco/releases/download/${MUJOCO_VERSION}/mujoco-${MUJOCO_VERSION}-linux-x86_64.tar.gz" \
      | tar -xz -C /opt \
    && ln -s "/opt/mujoco-${MUJOCO_VERSION}" /opt/mujoco
ENV CMAKE_PREFIX_PATH=/opt/mujoco
ENV LD_LIBRARY_PATH=/opt/mujoco/lib

# Pre-install Python runtime and build dependencies; cached in image layer so
# postCreate only compiles the C++ extension (~30 s) rather than downloading packages.
# Doc extras (sphinx / breathe / furo / sphinx-copybutton) are baked in so the
# postCreate docs build does not need network.
# chown lets the vscode user write to site-packages during postCreate's editable install.
RUN pip install --upgrade pip \
    && pip install "mujoco==3.3.2" "scikit-build-core>=0.12.2" "nanobind>=2.12.0" \
                   numpy scipy orjson \
                   sphinx sphinx-autobuild sphinx-copybutton breathe furo \
    && chown -R vscode:vscode /usr/local/lib/python3.12/site-packages
