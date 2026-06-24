FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

SHELL ["/bin/bash", "-lc"]
ARG DEBIAN_FRONTEND=noninteractive

ARG ROS_META_PKG=ros-jazzy-desktop
ARG ZED_ROS2_WRAPPER_TAG=humble-v5.1.0
ARG ROS_WS=/opt/ros_ws

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN apt-get update && apt-get install -y wget curl

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends locales; \
    locale-gen en_US en_US.UTF-8; \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends software-properties-common curl ca-certificates gnupg lsb-release; \
    add-apt-repository universe; \
    export ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')"; \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"; \
    dpkg -i /tmp/ros2-apt-source.deb; \
    rm -f /tmp/ros2-apt-source.deb; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    mkdir -p /etc/apt/keyrings; \
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
      -o /etc/apt/keyrings/githubcli-archive-keyring.gpg; \
    chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg; \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" \
      > /etc/apt/sources.list.d/github-cli.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends gh; \
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    apt-get update; \
    apt-get upgrade -y; \
    apt-get install -y --no-install-recommends \
      ${ROS_META_PKG} \
      ros-dev-tools \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
      python3-pip \
      build-essential \
      git \
      ros-jazzy-velodyne \
      ros-jazzy-rosbridge-server \
      ros-jazzy-tf-transformations \
      ros-jazzy-pointcloud-to-laserscan \
      ros-jazzy-tf2-tools \
      ros-jazzy-swri-console \
      ros-jazzy-cv-bridge \
      python3-transforms3d \
      python3-networkx \
      python3-serial \
      python3-bitstruct \
      xauth \
      tmux \
      neovim \
      emacs; \
    rm -rf /var/lib/apt/lists/*

# Install Ollama server/CLI binary
RUN curl -fsSL https://ollama.com/install.sh | sh

# Download Mistral model into the image
RUN ollama serve > /tmp/ollama.log 2>&1 & \
    sleep 5 && \
    ollama pull mistral && \
    pkill ollama

RUN echo "set -g history-limit 100000" >> /root/.tmux.conf && \
    echo "set -g mouse on" >> /root/.tmux.conf && \
    echo "set -g set-clipboard on" >> /root/.tmux.conf

RUN python3 -m pip uninstall --break-system-packages -y numpy || true

COPY requirements.txt /tmp/backend_requirements.txt
RUN python3 -m pip install --break-system-packages --ignore-installed --no-cache-dir -r /tmp/backend_requirements.txt

WORKDIR ${ROS_WS}/src

RUN git clone https://github.com/JACart2/anomaly_detection.git
RUN git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
RUN git clone --recursive --depth 1 --branch "${ZED_ROS2_WRAPPER_TAG}" https://github.com/stereolabs/zed-ros2-wrapper.git
RUN git clone https://github.com/stereolabs/zed-ros2-examples.git
RUN git clone https://github.com/rsasaki0109/lidar_localization_ros2.git

WORKDIR ${ROS_WS}

RUN rosdep init || true && rosdep update

RUN source /opt/ros/jazzy/setup.bash && \
    apt-get update -y || true && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --parallel-workers $(nproc) --symlink-install \
      --event-handlers console_direct+ \
      --base-paths src \
      --cmake-args \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
        -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"

RUN mkdir -p /maps && \
    wget -q -r -nd -l1 -A pcd -P /maps https://w3.cs.jmu.edu/spragunr/rosmaps/

RUN wget -O /root/student_auth_setup.sh https://raw.githubusercontent.com/JACart2/docker_files/main/student_auth_setup.sh && \
    chmod +x /root/student_auth_setup.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
