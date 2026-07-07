# syntax=docker/dockerfile:1.7
# Use the specified base image
FROM osrf/ros:jazzy-desktop

# needed for ros setup script on build
SHELL ["/bin/bash", "-lc"]

# Set environment variables to ensure correct behavior for installation and system setup
ENV DEBIAN_FRONTEND=noninteractive
ARG ROS_WS=/opt/ros_ws
ARG ANOMALY_DETECTION_REF=main

# nvidia-container-runtime & serial_number for cam
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,compute,video,utility

# general packages
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        python3-pip python3-dev python3-setuptools python3-tk \
        python3-rosdep python3-colcon-common-extensions \
        build-essential cmake git curl ca-certificates zstd \
        libgl1 libglib2.0-0 \
        ros-jazzy-velodyne \
        ros-jazzy-rosbridge-server \
        ros-jazzy-tf-transformations \
        ros-jazzy-pointcloud-to-laserscan \
        ros-jazzy-swri-console \
        python3-transforms3d \
        python3-networkx \
        python3-serial \
        python3-bitstruct \
        xauth \
        tmux \
        neovim \
        emacs; \
    rm -rf /var/lib/apt/lists/*

# Install the Ollama server/CLI used by the anomaly detection node.
RUN curl -fsSL https://ollama.com/install.sh | sh

# Download Mistral through a BuildKit cache, then copy it into the image.
RUN --mount=type=cache,target=/tmp/ollama-cache \
    set -eux; \
    export OLLAMA_MODELS=/tmp/ollama-cache/models; \
    ollama serve > /tmp/ollama.log 2>&1 & \
    ollama_pid="$!"; \
    for i in {1..30}; do \
        if ollama list >/dev/null 2>&1; then \
            break; \
        fi; \
        sleep 1; \
    done; \
    ollama pull mistral; \
    mkdir -p /root/.ollama; \
    rm -rf /root/.ollama/models; \
    cp -a /tmp/ollama-cache/models /root/.ollama/models; \
    kill "$ollama_pid"; \
    wait "$ollama_pid" || true
        

# Configure tmux
RUN echo "set -g history-limit 100000" >> /root/.tmux.conf && \
    echo "set -g mouse on" >> /root/.tmux.conf && \
    echo "set -g set-clipboard on" >> /root/.tmux.conf


# PREPPING DEV WORKSPACE----------------------------------------------------------------------------------------


RUN pip3 install  --break-system-packages --no-cache-dir numpy==1.26.4

## installing python packages from requirements.txt
COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed -r /tmp/requirements.txt

## Pulling in the anomaly_detection repo into the dev workspace
WORKDIR ${ROS_WS}/src

RUN git clone https://github.com/JACart2/anomaly_detection.git && \
    git -C anomaly_detection checkout "${ANOMALY_DETECTION_REF}" && \
    sed -i 's/<depend>built_interfaces<\/depend>/<depend>builtin_interfaces<\/depend>/' \
        anomaly_detection/anomaly_msg/package.xml

WORKDIR ${ROS_WS}

RUN set -ex; \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi; \
    rosdep update; \
    apt-get update; \
    rosdep install --from-paths src --ignore-src --skip-keys ament_python -r -y; \
    source /opt/ros/jazzy/setup.bash; \
    colcon build \
        --symlink-install \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=Release; \
    rm -rf /var/lib/apt/lists/*

# # initialize entrypoint, this gets access to the secret on the machine
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
