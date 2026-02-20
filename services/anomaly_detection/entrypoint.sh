#!/bin/bash
export OA_SECRET=$(cat /mnt/OA_SECRET.txt)

echo "OA_SECRET loaded (length: ${#OA_SECRET})"


# if no ANOMALY_DETECTION_COMMAND, default to compose.yml CMD. Otherwise, use ANOMALY_DETECTION_COMMAND
if [[ -z "$ANOMALY_DETECTION_COMMAND" ]]; then
  exec "$@"
else
  exec bash -lc "$ANOMALY_DETECTION_COMMAND"
fi

