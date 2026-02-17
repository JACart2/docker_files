# Use the specified base image
FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

# needed for ros setup script on build
SHELL ["/bin/bash", "-lc"]

# Set environment variables to ensure correct behavior for installation and system setup
ENV DEBIAN_FRONTEND=noninteractive

# Choose: ros-jazzy-ros-base (headless) or ros-jazzy-desktop (RViz etc.)
ARG ROS_META_PKG=ros-jazzy-desktop

# Pin the ZED ROS2 wrapper to match ZED SDK 5.1
# (Tags are in the repo; master expects SDK v5.1.) 
ARG ZED_ROS2_WRAPPER_TAG=humble-v5.1.0
ARG ROS_WS=/opt/ros_ws
ARG SHARED_ROS_RESOURCES=./shared_ros_resources

# nvidia-container-runtime & serial_number for cam
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility


# SETTING UP ROS2
# Get packages -------------------------------------------------------------------------------------------

# ---------------------------
# 1) Locale (per ROS docs for minimal envs like Docker)
# ---------------------------
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends locales; \
    locale-gen en_US en_US.UTF-8; \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8; \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# ---------------------------
# 2) Enable universe + install ros2-apt-source 
# ---------------------------
RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends software-properties-common curl ca-certificates gnupg lsb-release; \
    add-apt-repository universe; \
    \
    apt-get update; \
    apt-get install -y --no-install-recommends curl; \
    export ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')"; \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"; \
    dpkg -i /tmp/ros2-apt-source.deb; \
    rm -f /tmp/ros2-apt-source.deb; \
    rm -rf /var/lib/apt/lists/*

# ---------------------------
# 3) Install ROS 2 Jazzy + dev tools
# ---------------------------
RUN set -eux; \
    apt-get update; \
    apt-get upgrade -y; \
    apt-get install -y --no-install-recommends \
      ${ROS_META_PKG} \
      ros-dev-tools \
      python3-rosdep \
      python3-colcon-common-extensions \
      python3-vcstool \
      build-essential \
      git; \
    rm -rf /var/lib/apt/lists/*

# MISC OTHER PACKAGES----------------------------------------------------------------------------------------

RUN set -eux; \
    apt-get update; \
    apt-get install -y --no-install-recommends \
        python3-pip python3-dev python3-setuptools python3-tk \
        build-essential cmake git \
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
        

# Configure tmux
RUN echo "set -g history-limit 100000" >> /root/.tmux.conf && \
    echo "set -g mouse on" >> /root/.tmux.conf && \
    echo "set -g set-clipboard on" >> /root/.tmux.conf


# PREPPING DEV WORKSPACE----------------------------------------------------------------------------------------
    

# message type
COPY ${SHARED_ROS_RESOURCES}/anomaly_msg/ ${ROS_WS}/src/anomaly_msg/

# make /src
WORKDIR ${ROS_WS}

# Initialize rosdep
RUN rosdep init || true && rosdep update

# build
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
  apt-get update -y || true  && apt-get install -y ros-jazzy-rclpy || true && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "

WORKDIR ~
RUN . /opt/ros/jazzy/setup.sh 

# avoiding corruption with these after ros2 
## angry numpy version from base image, compat issues
RUN pip3 install --force-reinstall --break-system-packages --no-cache-dir numpy==1.26.4

## installing python packages from requirements.txt
COPY ./anomaly_detection/requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

## this should be the root of wherever manager.py will exist
WORKDIR /dev_ws/src/anomaly_detection

# # initialize entrypoint, this gets access to the secret on the machine
COPY ./anomaly_detection/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
