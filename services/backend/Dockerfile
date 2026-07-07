# syntax=docker/dockerfile:1.7
FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

SHELL ["/bin/bash", "-lc"]
ARG DEBIAN_FRONTEND=noninteractive

ARG ROS_META_PKG=ros-jazzy-desktop
ARG ZED_ROS2_WRAPPER_TAG=humble-v5.1.0
ARG ROS_WS=/opt/ros_ws
ARG ANOMALY_DETECTION_REF=901fdabd808629c17a11f5b700a6f16e798f87e7

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

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
    rm -rf /var/lib/apt/lists/*

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
      ${ROS_META_PKG} \
      gh \
      ros-dev-tools \
      python3-pip \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
      build-essential \
      ccache \
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
      emacs \
      python3-yaml \
      python3-empy \
      libflann-dev \
      liblz4-dev \
      libyaml-cpp-dev \
      libudev-dev \
      ros-jazzy-ament-cmake \
      ros-jazzy-ament-cmake-gtest \
      ros-jazzy-gtsam \
      ros-jazzy-mola \
      ros-jazzy-mola-common \
      ros-jazzy-mola-georeferencing \
      ros-jazzy-mola-gtsam-factors \
      ros-jazzy-mola-lidar-odometry \
      ros-jazzy-mola-state-estimation \
      ros-jazzy-mola-test-datasets \
      ros-jazzy-mola-viz \
      ros-jazzy-velodyne-driver \
      ros-jazzy-velodyne-pointcloud \
      ros-jazzy-rviz2 \
      wget \
      x11-xserver-utils; \
    rm -rf /var/lib/apt/lists/*

# Configure tmux
RUN echo "set -g history-limit 100000" >> /root/.tmux.conf && \
    echo "set -g mouse on" >> /root/.tmux.conf && \
    echo "set -g set-clipboard on" >> /root/.tmux.conf

RUN python3 -m pip uninstall --break-system-packages -y numpy || true

COPY requirements.txt /tmp/backend_requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    python3 -m pip install --break-system-packages --ignore-installed -r /tmp/backend_requirements.txt

WORKDIR ${ROS_WS}/src

RUN git clone --filter=blob:none --no-checkout https://github.com/JACart2/anomaly_detection.git /tmp/anomaly_detection && \
    git -C /tmp/anomaly_detection sparse-checkout init --cone && \
    git -C /tmp/anomaly_detection sparse-checkout set anomaly_msg && \
    git -C /tmp/anomaly_detection checkout --detach "${ANOMALY_DETECTION_REF}" && \
    mv /tmp/anomaly_detection/anomaly_msg ./anomaly_msg && \
    sed -i 's/<depend>built_interfaces<\/depend>/<depend>builtin_interfaces<\/depend>/' \
      anomaly_msg/package.xml && \
    rm -rf /tmp/anomaly_detection
RUN git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
RUN git clone --recursive --depth 1 --branch "${ZED_ROS2_WRAPPER_TAG}" https://github.com/stereolabs/zed-ros2-wrapper.git
RUN git clone https://github.com/stereolabs/zed-ros2-examples.git
RUN git clone https://github.com/rsasaki0109/lidar_localization_ros2.git

# Used for mapping and loop closure. This is a fork of the original MOLA repo with some modifications to make it work with ROS2.
RUN git clone https://github.com/MOLAorg/mola_sm_loop_closure.git src/mola_sm_loop_closure
RUN git -C src/mola_sm_loop_closure submodule update --init --recursive


# ============================================================================
# IMU SENSOR SUPPORT (InertialSense SDK)
# Comment out this entire section if IMU sensor is not needed.
# Also remove 'src/inertial-sense-sdk/ROS/ros2' from --base-paths in the
# colcon build step below.
# ============================================================================
ARG INERTIAL_SENSE_SDK_TAG=2.6.0
RUN git clone --recursive --branch "${INERTIAL_SENSE_SDK_TAG}" https://github.com/inertialsense/inertial-sense-sdk.git && \
  cd inertial-sense-sdk/ROS && \
  mkdir -p inertial-sense-sdk/scripts && \
  ln -sfn ../../../scripts/build_is_sdk.sh inertial-sense-sdk/scripts/build_is_sdk.sh

WORKDIR ${ROS_WS}

RUN rosdep init || true && rosdep update

# Get maps and Calibration Files
RUN mkdir -p /maps && \
    wget -q -r -np -nd -l1 -P /maps -A "*.yaml,*.pcd" https://w3.cs.jmu.edu/spragunr/rosmaps/

RUN wget -O /root/student_auth_setup.sh https://raw.githubusercontent.com/JACart2/docker_files/main/student_auth_setup.sh && \
    chmod +x /root/student_auth_setup.sh

# build
RUN --mount=type=cache,target=/root/.cache/ccache \
  set -eo pipefail; \
  export CCACHE_DIR=/root/.cache/ccache; \
  export CCACHE_MAXSIZE=5G; \
  source /opt/ros/jazzy/setup.bash; \
  apt-get update -y || true; \
  rosdep install --from-paths src --ignore-src -r -y; \
  colcon build --parallel-workers $(nproc) --symlink-install \
    --event-handlers console_direct+ \
    --base-paths src src/inertial-sense-sdk/ROS/ros2 \
    --cmake-args \
      -DBUILD_TESTING=OFF \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_FLAGS=-Wl,--allow-shlib-undefined


# Can these dependencies below be defined in the respective packages package.xml and then installed with rosdep?
#why transforms3d==0.4.1? on 06/17/2024 0.4.2 was released and caused issues. More specifically when running ros nodes "Please install transforms3d by hand" was outputed but doing so didn't fix anything.
# RUN . /opt/ros/jazzy/setup.sh \
#     && apt-get update \
#     && apt-get install -y python3-pip \
#     && pip3 install 'transforms3d==0.4.1' \
#     && apt-get install -y ros-jazzy-tf-transformations \
#     && apt install -y xauth \
#     && apt-get -y install tmux \
#     && chmod +x src/ai-navigation/motor_control/resource/docker_startup_script.sh \
#     && ./src/ai-navigation/motor_control/resource/docker_startup_script.sh

COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

WORKDIR ${ROS_WS}
