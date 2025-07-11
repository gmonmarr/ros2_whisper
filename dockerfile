FROM nvidia/cuda:12.2.0-devel-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ARG DEBIAN_FRONTEND=noninteractive

# Set up locale and install ROS 2 keys
RUN apt-get update && apt-get install -y \
    locales \
    curl \
    gnupg2 \
    tmux \
    lsb-release \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

# Add ROS 2 repo
RUN apt-get update && apt-get install -y software-properties-common && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    add-apt-repository universe && \
    echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 and build deps
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base python3-pip python3-colcon-common-extensions \
    python3-pyaudio portaudio19-dev alsa-utils python3-numpy \
    git build-essential wget curl nano tmux \
    ros-humble-rmw-fastrtps-cpp \
    cmake g++ pkg-config \
    && rm -rf /var/lib/apt/lists/*

# For color terminals (optional)
ENV TERM=xterm-256color

# Install pip dependencies for your demos
RUN pip3 install --upgrade pip && \
    pip3 install pynput webrtcvad

# --- Custom: tmux mouse mode ---
RUN echo "set -g mouse on" > /root/.tmux.conf

# --- Custom: Download Whisper base model ---
RUN mkdir -p /root/.cache/whisper.cpp && \
    cd /root/.cache/whisper.cpp && \
    wget -4 -O ggml-base.en.bin https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin

# Set up ROS workspace and clone the repo
WORKDIR /workspace/ros-ai
RUN mkdir -p src && \
    git clone https://github.com/gmonmarr/ros2_whisper.git src/ros2_whisper

# CUDA env for CMake etc
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/cuda/bin:${PATH}

# Limit CMake parallelism for RAM-starved systems (adjust if you have more memory!)
ENV CMAKE_BUILD_PARALLEL_LEVEL=2

# Build the workspace
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --cmake-args -DGGML_CUDA=On --no-warn-unused-cli

# For running audio input (allow access to audio devices)
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native
ENV XDG_RUNTIME_DIR=/run/user/1000
ENV ROS_DOMAIN_ID=42

# Entrypoint (source ROS 2 and workspace)
SHELL ["/bin/bash", "-c"]
ENTRYPOINT source /opt/ros/humble/setup.bash && source install/setup.bash && bash
