# Use the specified base image
FROM stereolabs/zed:4.2-runtime-cuda12.1-ubuntu22.04

# Set environment variables to ensure correct behavior for installation and system setup
ENV DEBIAN_FRONTEND=noninteractive


# Update package lists and install dependencies
RUN apt-get update && \
  apt-get install -y python3-pip && \
  apt-get install -y python3-dev && \
  apt-get install -y python3-setuptools && \
  apt-get install -y python3-tk && \
  apt-get install -y build-essential && \
  apt-get install -y cmake && \
  apt-get install -y git && \
  apt-get install -y cmake && \
  apt-get install -y libgl1-mesa-glx && \
  apt install -y libglib2.0-0 && \
  rm -rf /var/lib/apt/lists/* 

# nvidia-container-runtime & serial_number for cam
ENV NVIDIA_VISIBLE_DEVICES=${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES=${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics,compute,video,utility

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

# Install rosbridge so ui can communicate with ros
RUN apt-get install -y ros-humble-rosbridge-server

# NVIDIA install
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-ubuntu2204.pin && \
  mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600 && \
  wget https://developer.download.nvidia.com/compute/cuda/12.5.0/local_installers/cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb && \
  dpkg -i cuda-repo-ubuntu2204-12-5-local_12.5.0-555.42.02-1_amd64.deb && \ 
  cp /var/cuda-repo-ubuntu2204-12-5-local/cuda-*-keyring.gpg /usr/share/keyrings/ && \
  apt-get update && \
  apt-get -y install cuda-toolkit-12-5 && \
  apt-get install -y cuda-drivers


# Setup Zed SDK https://www.stereolabs.com/docs/installation/linux ---------------------------------------------------
#zed cuda dependency (https://developer.nvidia.com/cuda-downloads)
RUN wget -O zed-sdk.run https://download.stereolabs.com/zedsdk/4.0/cu117/ubuntu22 && \
  apt-get install -y libxml2 && \
  apt-get install -y gcc && \
  apt-get install -y zstd && \
  sh zed-sdk.run -- silent skip_tools skip_cuda 

# SETTING UP ROS2
# Get packages -------------------------------------------------------------------------------------------

# WORKDIR /dev_ws/src
# RUN git clone --recursive --depth 1 --branch humble-v4.0.8 https://github.com/stereolabs/zed-ros2-wrapper.git && \
#   git clone https://github.com/stereolabs/zed-ros2-examples.git && \
#   apt-get update

# make /src
WORKDIR /dev_ws/src
WORKDIR /dev_ws
# Initialize rosdep
RUN rosdep init && rosdep update


# build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
  apt-get update -y || true && \
  rosdep install --from-paths src --ignore-src -r -y && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "

RUN . /opt/ros/humble/setup.sh \
  && apt-get update \
  && pip3 install 'transforms3d==0.4.1' \
  && apt-get install -y ros-humble-tf-transformations \
  && apt install -y xauth \
  && apt-get -y install tmux 


# avoiding corruption with these after ros2 
# Install the FER package (using pip)
RUN pip3 install fer 

# Hack fix because Numpy installation is corrupted. I believe this is because of the zed-sdk installation above, but not 100% sure.
RUN rm -rf /usr/local/lib/python3.10/dist-packages/numpy*
RUN pip install numpy --force-reinstall

RUN pip3 install moviepy==1.0.3 && \
  pip3 install opencv-python && \
  pip3 install tensorflow && \
  pip3 install ffmpeg && \
  pip3 install argon2-cffi && \ 
  pip3 install python-dotenv && \
  pip3 install openai && \
  pip3 install mysql-connector-python


WORKDIR /dev_ws/src/FER_DataCollection/modified_legacy_code

# # initialize entrypoint, this gets access to the secret on the machine
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]