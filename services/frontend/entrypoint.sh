#!/bin/bash
set -e

# Match docker-compose working_dir
cd /root/ui

# Optional: update if it's a git repo
if [ -d ".git" ]; then
  echo "Updating UI from GitHub..."
  git pull origin main || echo "Warning: Git pull failed, using existing code."
fi

echo "Installing npm dependencies..."
npm ci --prefer-offline --no-audit

if [[ -n "$FRONTEND_COMMAND" ]]; then
  exec bash -lc "$FRONTEND_COMMAND"
else
  exec "$@"
fi
