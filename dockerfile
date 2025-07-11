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
    tree \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG en_US.UTF-8

# Add ROS 2 repository
RUN apt-get update && apt-get install -y software-properties-common && \
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | apt-key add - && \
    add-apt-repository universe && \
    echo "deb [arch=amd64] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list

# Install ROS 2 and dev tools
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    python3-pip \
    python3-colcon-common-extensions \
    python3-pyaudio \
    portaudio19-dev \
    alsa-utils \
    python3-numpy \
    git \
    build-essential \
    wget \
    curl \
    nano \
    tmux \
    ros-humble-rmw-fastrtps-cpp \
    cmake \
    g++ \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# For color terminal
ENV TERM=xterm-256color

# Python dependencies for demos
RUN pip3 install --upgrade pip && \
    pip3 install pynput webrtcvad

# tmux mouse mode
RUN echo "set -g mouse on" > /root/.tmux.conf

# Download Whisper model (optional)
RUN mkdir -p /root/.cache/whisper.cpp && \
    cd /root/.cache/whisper.cpp && \
    wget -4 -O ggml-base.en.bin https://huggingface.co/ggerganov/whisper.cpp/resolve/main/ggml-base.en.bin

# Set working directory for ROS workspace
WORKDIR /workspace/ros-ai

# CUDA and ROS environment
ENV LD_LIBRARY_PATH=/usr/local/cuda/lib64:${LD_LIBRARY_PATH}
ENV PATH=/usr/local/cuda/bin:${PATH}
ENV CMAKE_BUILD_PARALLEL_LEVEL=2
ENV PULSE_SERVER=unix:/run/user/1000/pulse/native
ENV XDG_RUNTIME_DIR=/run/user/1000
ENV ROS_DOMAIN_ID=42

# Entrypoint
SHELL ["/bin/bash", "-c"]
ENTRYPOINT source /opt/ros/humble/setup.bash && bash
