#!/bin/sh
export OA_SECRET=$(cat /mnt/OA_SECRET.txt)

echo "OA_SECRET is set to: $OA_SECRET"
echo "SERIAL_NUMBER is set to: $SERIAL_NUMBER"

echo "RUN_FER is set to: $RUN_FER"
if [ "$RUN_FER" == "false"  ]; then
    echo "NOT RUNNING FER"
else
    echo "RUNNING FER"

    # this will run the emotiondetection script
    cd FER_DataCollection
    cd modified_legacy_code
    python3 EmotionDetection.py
fi
