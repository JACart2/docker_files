FROM stereolabs/zed:5.1-gl-devel-cuda12.8-ubuntu24.04

SHELL ["/bin/bash", "-lc"]
ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y wget

# Choose: ros-jazzy-ros-base (headless) or ros-jazzy-desktop (RViz etc.)
ARG ROS_META_PKG=ros-jazzy-desktop

# Pin the ZED ROS2 wrapper to match ZED SDK 5.1
# (Tags are in the repo; master expects SDK v5.1.) 
ARG ZED_ROS2_WRAPPER_TAG=humble-v5.1.0
ARG ROS_WS=/opt/ros_ws

# INSTALL ROS----------------------------------------------------------------------------------------

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
    apt-get install -y ros-jazzy-velodyne \
    ros-jazzy-rosbridge-server \
    ros-jazzy-tf-transformations \
    ros-jazzy-pointcloud-to-laserscan \
    python3-transforms3d \
    python3-networkx \
    python3-serial \
    python3-bitstruct \
    ros-jazzy-swri-console \
    xauth \
    tmux \
    neovim \
    emacs 

# Configure tmux
RUN echo "set -g history-limit 100000" >> /root/.tmux.conf && \
    echo "set -g mouse on" >> /root/.tmux.conf && \
    echo "set -g set-clipboard on" >> /root/.tmux.conf

# It seems that the base image includes a pip installed version of numpy
# that causes all sorts of compatibility issues
RUN python3 -m pip uninstall --break-system-packages -y numpy

# Get packages -------------------------------------------------------------------------------------------
WORKDIR ${ROS_WS}/src
RUN git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
RUN git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
#RUN git clone --branch display_global_path https://github.com/JACart2/ai-navigation.git
RUN git clone --recursive --depth 1 --branch "${ZED_ROS2_WRAPPER_TAG}" https://github.com/stereolabs/zed-ros2-wrapper.git 
RUN git clone https://github.com/stereolabs/zed-ros2-examples.git
#RUN git clone --branch ros2 https://github.com/JACart2/LIO-SAM.git

WORKDIR ${ROS_WS}


# Initialize rosdep
RUN rosdep init && rosdep update

# build
RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
  apt-get update -y || true && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "


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

# Get maps
RUN mkdir -p /maps && \
    wget -q -r -nd -l1 -A pcd -P /maps https://w3.cs.jmu.edu/spragunr/rosmaps/

# setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
