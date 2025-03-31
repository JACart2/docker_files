# Run this one time when first creating the dev_ws folder on a new computer

# Get packages -------------------------------------------------------------------------------------------
mkdir -p src
cd src
git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
git clone https://github.com/rsasaki0109/ndt_omp_ros2.git
git clone --branch display_global_path https://github.com/JACart2/ai-navigation.git
git clone --recursive --depth 1 --branch humble-v4.0.8 https://github.com/stereolabs/zed-ros2-wrapper.git
git clone https://github.com/stereolabs/zed-ros2-examples.git
# apt-get update
# apt-get -y install ros-humble-velodyne
cd ../dev_ws
