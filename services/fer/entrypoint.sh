#!/bin/sh
export OA_SECRET=$(cat /mnt/OA_SECRET.txt)

echo "OA_SECRET is set to: $OA_SECRET"

# setup ros2 environment
source "/opt/ros/humble/setup.bash" --
source "/dev_ws/install/setup.bash" --
echo 'source "/opt/ros/humble/setup.bash" ' >> ~/.bashrc 
echo 'source "/dev_ws/install/setup.bash" ' >> ~/.bashrc 

# take in command line input and execute

if [[ -z "$FER_COMMAND" ]]; then
tail -f /dev/null
#     echo "RUNNING FER"

#     # this will run the emotiondetection script
#     cd ~/src/FER_DataCollection
#     cd modified_legacy_code
#     python3 EmotionDetection.py
else
  bash -c "$FER_COMMAND"
fi

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
