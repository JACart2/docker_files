#!/usr/bin/env bash
set -euo pipefail

# Cart host bootstrap for Ubuntu 24.04 (Noble)
# Installs: VS Code, Google Chrome, gh, Docker, NVIDIA Container Toolkit
# Adds udev rule: /etc/udev/rules.d/99-usb-serial.rules

if [[ "$(lsb_release -rs 2>/dev/null || true)" != "24.04" ]]; then
  echo "WARNING: This script is intended for Ubuntu 24.04. Continuing anyway..."
fi

echo "==> Updating apt indices"
sudo apt-get update -y
sudo apt-get install -y ca-certificates curl gnupg lsb-release wget software-properties-common

sudo install -m 0755 -d /etc/apt/keyrings

# -------------------------
# VS Code (Microsoft repo)
# -------------------------
echo "==> Installing VS Code repo + package"
# From Microsoft VS Code Linux setup docs (apt repo) :contentReference[oaicite:5]{index=5}
curl -fsSL https://packages.microsoft.com/keys/microsoft.asc \
  | sudo gpg --dearmor -o /etc/apt/keyrings/microsoft.gpg
sudo chmod a+r /etc/apt/keyrings/microsoft.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/microsoft.gpg] \
https://packages.microsoft.com/repos/code stable main" \
| sudo tee /etc/apt/sources.list.d/vscode.list > /dev/null

# -------------------------
# GitHub CLI (gh) repo
# -------------------------
echo "==> Installing GitHub CLI (gh) repo + package"
# From GitHub CLI install docs :contentReference[oaicite:6]{index=6}
curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg \
  | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] \
https://cli.github.com/packages stable main" \
| sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null

# -------------------------
# Docker Engine repo
# -------------------------
echo "==> Installing Docker repo + packages"
# From Docker Engine install guide for Ubuntu (use noble) :contentReference[oaicite:7]{index=7}
curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
https://download.docker.com/linux/ubuntu noble stable" \
| sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# -------------------------
# NVIDIA Container Toolkit repo
# -------------------------
echo "==> Installing NVIDIA Container Toolkit repo"
# From NVIDIA Container Toolkit install guide :contentReference[oaicite:8]{index=8}
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /etc/apt/keyrings/nvidia-container-toolkit.gpg
sudo chmod a+r /etc/apt/keyrings/nvidia-container-toolkit.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/etc/apt/keyrings/nvidia-container-toolkit.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

# -------------------------
# Install packages
# -------------------------
echo "==> Installing apt packages (code, gh, docker, nvidia-container-toolkit)"
sudo apt-get update -y
sudo apt-get install -y \
  code \
  gh \
  docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin \
  nvidia-container-toolkit

# -------------------------
# Install Google Chrome (download .deb)
# -------------------------
echo "==> Installing Google Chrome"
# Common official method: download .deb then install :contentReference[oaicite:9]{index=9}
tmpdir="$(mktemp -d)"
trap 'rm -rf "$tmpdir"' EXIT
(
  cd "$tmpdir"
  wget -q https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
  sudo apt-get install -y ./google-chrome-stable_current_amd64.deb
)

# -------------------------
# Configure Docker + user group
# -------------------------
echo "==> Enabling Docker service"
sudo systemctl enable --now docker

echo "==> Adding current user to docker group (may require logout/login)"
sudo usermod -aG docker "$USER" || true

# -------------------------
# Configure NVIDIA runtime for Docker
# -------------------------
echo "==> Configuring NVIDIA Container Toolkit for Docker"
# nvidia-ctk runtime configure + restart docker :contentReference[oaicite:10]{index=10}
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# -------------------------
# Install udev rules
# -------------------------
echo "==> Installing udev rules (/etc/udev/rules.d/99-usb-serial.rules)"
sudo tee /etc/udev/rules.d/99-usb-serial.rules > /dev/null <<'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", SYMLINK+="ttyUSB9"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="rplidar"
EOF

echo "==> Reloading udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger

# -------------------------
# Quick verification hints
# -------------------------
echo
echo "==> Done."
echo "Notes:"
echo "  - You may need to log out/in for docker group changes to take effect."
echo "  - To test Docker:        docker run --rm hello-world"
echo "  - To test NVIDIA Docker: docker run --rm --gpus all nvidia/cuda:12.8.0-base-ubuntu24.04 nvidia-smi"
echo "  - udev rules installed:  /etc/udev/rules.d/99-usb-serial.rules"
