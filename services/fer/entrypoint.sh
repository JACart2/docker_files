#!/bin/sh
export OA_SECRET=$(cat /mnt/OA_SECRET.txt)

echo "OA_SECRET is set to: $OA_SECRET"
echo "SERIAL_NUMBER is set to: $SERIAL_NUMBER"

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/dev_ws/install/setup.bash" --
echo 'source "/opt/ros/humble/setup.bash" ' >> ~/.bashrc 
echo 'source "/dev_ws/install/setup.bash" ' >> ~/.bashrc 

# echo "RUN_FER is set to: $RUN_FER"
# if [ "$RUN_FER" == "true"  ]; then
#     echo "RUNNING FER"

#     # this will run the emotiondetection script
#     cd ~/src/FER_DataCollection
#     cd modified_legacy_code
#     python3 EmotionDetection.py
# else
#     echo "NOT RUNNING FER"
# fi
