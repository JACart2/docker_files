FROM ubuntu:22.04
RUN apt-get update && apt-get install -y \
 	wget
ARG DEBIAN_FRONTEND=noninteractive

# ROS2 https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html -------------------------------------------------------
RUN apt update
# Set and Check Locale
RUN apt-get install --no-install-recommends -y locales && \
	locale-gen en_US en_US.UTF-8 && \
	update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
	export LANG=en_US.UTF-8 && \
	locale

# Setup Sources
RUN apt-get install --no-install-recommends -y software-properties-common && \
	add-apt-repository universe && \
	apt-get install -y curl && \
	curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null 
# Install ROS 2 Base packages and Python dependencies
RUN apt-get update && apt-get install --no-install-recommends -y \
	ros-humble-desktop \
	ros-dev-tools
# Setup Cuda https://developer.nvidia.com/cuda-downloads ------------------------------------------------------------

RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin
RUN mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
RUN wget https://developer.download.nvidia.com/compute/cuda/12.5.0/local_installers/cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb
RUN dpkg -i cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb
RUN cp /var/cuda-repo-ubuntu2204-12-5-local/cuda-*-keyring.gpg /usr/share/keyrings/
RUN apt-get update
RUN apt-get -y install cuda-toolkit-12-5
RUN apt-get install -y cuda-drivers

# Setup Zed SDK https://www.stereolabs.com/docs/installation/linux ---------------------------------------------------
RUN wget -O zed-sdk.run https://download.stereolabs.com/zedsdk/4.0/cu117/ubuntu22
#zed cuda dependency (https://developer.nvidia.com/cuda-downloads)
RUN apt-get install -y libxml2
RUN apt-get install -y gcc
RUN apt-get install -y zstd
RUN sh zed-sdk.run -- silent skip_tools skip_cuda

# Install rosbridge so ui can communicate with ros
RUN apt-get install -y ros-humble-rosbridge-server

# Install GT-SAM
RUN apt-get update && apt-get install -y ros-humble-gtsam
# RUN apt-get update && apt-get install -y libgtsam-dev

# Install G20
# RUN apt-get update && apt-get install -y libg2o-dev
# Install dependencies required for building g2o
RUN apt-get update && apt-get install -y \
    cmake \
    build-essential \
    git \
    libeigen3-dev \
    libsuitesparse-dev \
    qt5-qmake \
    libqt5widgets5

# Clone and build g2o
RUN git clone https://github.com/RainerKuemmerle/g2o.git /root/g2o && \
    cd /root/g2o && \
    mkdir build && cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility

# setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod 755 /entrypoint.sh

# Environment installs
RUN . /opt/ros/humble/setup.sh \
    && sudo apt-get update \
    && sudo apt-get install -y python3-pip \
    && pip3 install 'transforms3d==0.4.1' \
    && sudo apt-get install -y ros-humble-tf-transformations \
    && sudo apt install -y xauth

# Install and build ros dependencies
WORKDIR /dependency_ws/src
RUN git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
RUN git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
RUN git clone --recursive --depth 1 --branch humble-v4.0.8 https://github.com/stereolabs/zed-ros2-wrapper.git
RUN git clone https://github.com/stereolabs/zed-ros2-examples.git
RUN git clone https://github.com/stereolabs/zed-ros2-interfaces.git


WORKDIR /dependency_ws
RUN bash -c "source /opt/ros/humble/setup.bash && \
	     rosdep init && rosdep update && rosdep install --from-paths /dependency_ws/src --ignore-src -r -y && \
	     colcon build --symlink-install"

# brltty is a package that conflicts with the driver that arudino uses for its usb connection
RUN apt remove brltty

# install neovim for development in container
RUN apt-get install -y neovim
RUN apt-get install -y tmux
RUN sudo apt-get install libpcl-dev

# Install Velodyne ROS 2 driver
RUN apt-get update && apt-get install -y \
    ros-humble-velodyne-driver \
    ros-humble-velodyne-msgs \
    ros-humble-velodyne-pointcloud

# LIO-SAM dependencies
RUN apt-get update && apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-libpointmatcher \
    ros-humble-tf2-sensor-msgs \
    libproj-dev \
    libeigen3-dev

# Ensure workspace directory exists
RUN mkdir -p /dev_ws/src

ENTRYPOINT ["/entrypoint.sh"]