#!/bin/bash

UI_DIR="/root/dev_ws/src/ui"

if [[ -z "$FRONTEND_COMMAND" ]]; then
  exec "$@"
else
  # Update UI code from GitHub (if ui is a git repo)
  if [ -d "$UI_DIR/.git" ]; then
    echo "Updating UI from GitHub..."
    cd "$UI_DIR"
    git pull origin main || echo "Warning: Git pull failed, using existing code."
    npm install || echo "Warning: npm install failed."
  else
    echo "Error: $UI_DIR is not a Git repository. Cannot update UI."
  fi

  # Execute the frontend command
  bash -c "$FRONTEND_COMMAND"
fi
