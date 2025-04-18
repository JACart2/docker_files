#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/dependency_ws/install/setup.bash"
cd /dev_ws
rm -rf build log install
rosdep update
rosdep install --from-paths /dev_ws/src --ignore-src -r -y 
colcon build --symlink-install
source /dev_ws/install/setup.bash
echo 'source "/opt/ros/humble/setup.bash" ' >> ~/.bashrc 
echo 'source "/dev_ws/install/setup.bash" ' >> ~/.bashrc 

# Environment settings that need to reference source code

PKG="/dev_ws/src/ai-navigation/motor_control"

# sets up a rule file that automatically sets the arduino to port "ttyUSB9" everytime
cp "${PKG}/resource/99-usb-serial.rules" "/etc/udev/rules.d"

# installs all required packages
pip install -r ${PKG}/resource/requirements.txt

# take in command line input and execute

if [[ -z "$BACKEND_COMMAND" ]]; then
  exec "$@"
else
  bash -c "$BACKEND_COMMAND"
fi

