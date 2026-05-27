# Use Ubuntu as base image
FROM ubuntu:22.04

# Non-interactive installation
ENV DEBIAN_FRONTEND=noninteractive
ENV BUILD_OUTPUT_DIR=/workspace/build

# Install build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    ninja-build \
    git \
    curl \
    tar \
    libgl1-mesa-dev \
    libxinerama-dev \
    libxcursor-dev \
    libxrandr-dev \
    libxi-dev \
    libboost-all-dev \
    libgtest-dev \
    flatbuffers-compiler \
    libflatbuffers-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Install Mujoco
RUN git clone https://github.com/deepmind/mujoco.git \
    && cd mujoco \
    && git checkout tags/3.3.2 \
    && cmake -S . -B build -DMUJOCO_BUILD_EXAMPLES=OFF -DMUJOCO_BUILD_SIMULATE=OFF -DMUJOCO_BUILD_TESTS=OFF -DBUILD_TESTING=OFF \
    && cmake --build build --target install -- -j$(nproc)

# Copy project files into container
COPY . /workspace

# Configure and build the project
# RUN cmake -S . -B $BUILD_OUTPUT_DIR \
#    -DCMAKE_C_COMPILER=gcc \
#    -DCMAKE_CXX_COMPILER=g++ \
#    -DCMAKE_BUILD_TYPE=Release \
#    -DBUILD_TESTS=ON \
#    && cmake --build $BUILD_OUTPUT_DIR -- -j$(nproc)

# Run tests
WORKDIR $BUILD_OUTPUT_DIR

CMD ["/bin/bash"]
# RUN ctest --output-on-failure
