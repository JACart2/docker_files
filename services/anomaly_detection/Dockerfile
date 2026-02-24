# Use the specified base image
FROM osrf/ros:jazzy-desktop 

# needed for ros setup script on build
SHELL ["/bin/bash", "-lc"]

# Set environment variables to ensure correct behavior for installation and system setup
ENV DEBIAN_FRONTEND=noninteractive
# Choose: ros-jazzy-ros-base (headless) or ros-jazzy-desktop (RViz etc.)
ARG ROS_META_PKG=ros-jazzy-desktop

# nvidia-container-runtime & serial_number for cam
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility

# general packages
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


RUN pip3 install  --break-system-packages --no-cache-dir numpy==1.26.4

## installing python packages from requirements.txt
COPY ./requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

## this should be the root of anomaly_detection, 
WORKDIR /root/dev_ws/src/anomaly_detection

# # initialize entrypoint, this gets access to the secret on the machine
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
