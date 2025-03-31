# Run this one time when first creating the dev_ws folder on a new computer

# Get packages -------------------------------------------------------------------------------------------
echo "Cloning repos"
mkdir -p src
cd src
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
git clone --branch display_global_path https://github.com/JACart2/ai-navigation.git
git clone --recursive --depth 1 --branch humble-v4.0.8 https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-examples.git
git clone https://github.com/stereolabs/zed-ros2-interfaces.git
sudo apt-get update
sudo apt-get -y install ros-humble-velodyne
cd ..


# Initialize rosdep
echo "Initializing rosdep"
rosdep init && rosdep update

# build
echo "Sourcing and building"
source /opt/ros/humble/setup.bash && \
   sudo apt-get update -y || true && \
   rosdep install --from-paths src --ignore-src -r -y && \
   colcon build --parallel-workers $(nproc) --symlink-install \
   --event-handlers console_direct+ --base-paths src \
   --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
   ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
   ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"'

# /bin/bash -c "source /opt/ros/humble/setup.bash && \
#   apt-get update -y || true && \
#   rosdep install --from-paths src --ignore-src -r -y && \
#   colcon build --parallel-workers $(nproc) --symlink-install \
#   --event-handlers console_direct+ --base-paths src \
#   --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
#   ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
#   ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' "

echo "Installing other dependencies"
# Can these dependencies below be defined in the respective packages package.xml and then installed with rosdep?
#why transforms3d==0.4.1? on 06/17/2024 0.4.2 was released and caused issues. More specifically when running ros nodes "Please install transforms3d by hand" was outputed but doing so didn't fix anything.
. /opt/ros/humble/setup.sh \
    && sudo apt-get update \
    && sudo apt-get install -y python3-pip \
    && pip3 install 'transforms3d==0.4.1' \
    && sudo apt-get install -y ros-humble-tf-transformations \
    && sudo apt install -y xauth \
    && sudo apt-get -y install tmux \
    && chmod +x src/ai-navigation/motor_control/resource/docker_startup_script.sh \
    && ./src/ai-navigation/motor_control/resource/docker_startup_script.sh
