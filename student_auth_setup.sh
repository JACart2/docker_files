#!/usr/bin/env bash
set -euo pipefail

# This script sets up the git identity for the current session on a shared machine.
# It authenticates with GitHub and sets the global git configuration for the user.
# NOTE: Since this is a shared account, users must remember to logout/clear this when done.

echo "=== Student Git Session Setup ==="
echo

# 1. Get User Info
read -r -p "Enter your Full Name (for git commits): " GIT_NAME
read -r -p "Enter your Email (for git commits): " GIT_EMAIL

if [[ -z "${GIT_NAME}" || -z "${GIT_EMAIL}" ]]; then
  echo "ERROR: Name and Email are required."
  exit 1
fi

echo
echo "Configuring global git identity..."
# Set global config so it applies to any repo they touch during this session
git config --global user.name "${GIT_NAME}"
git config --global user.email "${GIT_EMAIL}"
echo "Git identity set to: ${GIT_NAME} <${GIT_EMAIL}>"

echo
echo "Authenticating with GitHub..."
# Login to GitHub CLI (interactive)
if command -v gh >/dev/null 2>&1; then
    gh auth login
else
    echo "Warning: 'gh' (GitHub CLI) not found. Skipping GitHub authentication."
fi

echo
echo "=== Setup Complete ==="
echo "You are now ready to commit and push code."
echo "IMPORTANT: accurate attribution depends on you running this script."
echo "When finished, please run: gh auth logoutAnd unset your git config if you wish."
