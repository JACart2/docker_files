# JACart Docker Environment

Docker setup for running the JACart software stack using **ROS 2 Jazzy** on **Ubuntu 24.04**.

## Prerequisites
*   **OS**: Ubuntu 24.04 (Noble Numbat)
*   **Hardware**: x86_64 system with NVIDIA GPU (CUDA capable)

## Setup

### Part 1: Host Machine Setup (One-time Admin Task)
*Perform this only once when configuring a fresh machine.*

1.  Clone this repository.
2.  Run the host bootstrap script to install system dependencies (Docker, NVIDIA drivers, VS Code, etc.):
    ```bash
    ./setup_cart_host_ubuntu24.sh
    ```
3.  **Reboot** the machine if NVIDIA drivers were installed or updated.

### Part 2: User Account Setup
*Perform this for every user account that will use this software.*

1.  Clone this repository (if you haven't already).
2.  Clone the `ai_navigation` repository into `~/dev_ws`:
    ```bash
    mkdir -p ~/dev_ws/src
    cd ~/dev_ws/src
    git clone https://github.com/JACart2/ai-navigation.git
    ```
3.  Add your user to the docker group to run containers without `sudo`:
    ```bash
    sudo usermod -aG docker $USER
    newgrp docker
    ```

## Usage

### Production / Full Run
To start the full system (Backend ROS nodes + Frontend UI):
```bash
./run.sh
```

### Development
To start the containers in a detached state and attach an interactive terminal to the backend:

**Option 1: Backend Development**
```bash
./dev-run-backend.sh
```
*   **Backend**: Stalled (interactive terminal attached).
*   **Frontend**: Runs normally (web UI accessible at localhost:5173).
*   Mounts your local workspace (default: `$HOME/dev_ws`) to `/dev_ws`.
*   Terminal starts in `/dev_ws` with all ROS environments sourced.

**Option 2: Full Debugging**
```bash
./dev-run.sh
```
*   **Backend**: Stalled (interactive terminal attached).
*   **Frontend**: Stalled (does not start npm/webserver automatically).
*   Use this if you need to debug the frontend startup process manually.

## Utility Scripts
*   `setup_cart_host_ubuntu24.sh`: One-stop setup for a fresh Ubuntu 24.04 machine.
*   `velodyne-net-setup.sh`: Sets up a static IP for the Velodyne LIDAR subnet (default: 192.168.1.254/24) on a suitable ethernet interface and configures a persistent systemd service to apply this at boot.

    > **Warning:** This will make the selected ethernet port unavailable for internet access while configured for the Velodyne network.

    You can override the default interface and IP using the `IFACE` and `HOST_IP` environment variables:
    ```bash
    IFACE=enp0s31f6 HOST_IP=192.168.1.100/24 ./scripts/velodyne-net-setup.sh
    ```
    After running, the network configuration will persist across reboots.
*   `initialize_host.sh`: Configures host network headers for Velodyne (IP 192.168.1.254) and X11 display permissions. **Run automatically** by start scripts.
*   `compose.yaml`: Defines the `backend` (ROS 2) and `frontend` (Web UI) services running in host network mode.
*   `dockerTeardown.sh`: Forcefully stops running containers.
*   `student_auth_setup.sh`: Helper for students on shared machines to configure their Git identity and authenticate with GitHub for the session.

### VSCode Integration

Install the [VSCode Devcontainers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). This will enable you to code inside the containers. If you create the containers (using `dev-run-backend.sh` or `dev-run.sh`) and open the remote explorer tab in VSCode you should see the frontend and backend containers. Here is what it looks like:

![devcontainers](https://github.com/user-attachments/assets/f1954457-1171-4320-a687-2cc7833905c9)

Once you open the container in vscode, opening a terminal (through VSCode) will also give you access to the development container.

## Useful Docker Commands
- `docker run -it <image> /bin/bash`                :    Launch an image with a bash terminal (Simple way to check file structure)
- `docker exec -it <container id> /bin/bash`        :    Open a bash terminal inside a running container
- `docker container ls -a`                          :    List all containers (regardless of status)
- `docker images`                                   :    List all image details
- `docker build -t <image_name> <dockerfile_path>`  :    Manually rebuild a container
- `docker start -ai <container id>`                 :    Manually start a container

## Manual Clean Up
- `docker rm <Container id or name>`  :  Remove specified container
- `docker rmi <Image id or name>`     :  Remove specified image

---

## Launching the cart with radar

The fastest "I want the cart running" loop is the dev-backend script plus a launch line:

```bash
# 1. From the host (this repo's directory), bring up the backend in stalled
#    mode and drop into a shell inside it. Frontend comes up normally on
#    http://localhost:5173.
./dev-run-backend.sh

# 2. Inside the backend shell, launch the full autonomous stack with the
#    cart's actual serial-port IDs (default ttyUSB0/ttyUSB1 don't match
#    this hardware):
ros2 launch cart_launch autonomous_launcher.launch.py \
  motor_arduino_port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00448R-if00-port0 \
  radar_command_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if00-port0 \
  radar_data_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if01-port0
```

This brings up everything in a single launch: TI mmWave radar driver, `radar_xyz_filter` (with Tk slider window), `radar_pcl_to_obstacles`, velodyne + lidar localization, navigation stack, motor endpoint, rosbridge, swri_console, and **two RViz windows** (top-down map view + radar-perspective view). See the [ai-navigation README](https://github.com/JACart2/ai-navigation) for what each radar node does and how to retune the filter.

### What's in this repo for the radar setup

* `compose.yaml` — has `BROWSER=none` in the frontend `environment:` so vite doesn't try to spawn `xdg-open` inside the browser-less container and crash on startup. The `ui` repo's `vite.config.ts` ships `server.open: true` (regression introduced upstream in `ui` commit `60b555b0`, 2025-02-11), so this env var is what keeps the frontend container alive. Don't remove it without also fixing the upstream config.
* `services/backend/Dockerfile` — patches the TI radar's `6843AOP_Standard.cfg` at image-build time to set `clutterRemoval -1 0`, keeping short-range returns the radar would otherwise filter out.

### `run.sh` quirk

The "production" `./run.sh` runs the backend container with the compose default `command: ros2 launch cart_launch autonomous_launcher.launch.py` — which uses `/dev/ttyUSB0` / `/dev/ttyUSB1`, not the cart's by-id paths. If `run.sh` doesn't bring the hardware up, override the backend command for that run:

```bash
BACKEND_COMMAND="ros2 launch cart_launch autonomous_launcher.launch.py \
  motor_arduino_port:=/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG00448R-if00-port0 \
  radar_command_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if00-port0 \
  radar_data_port:=/dev/serial/by-id/usb-Silicon_Labs_CP2105_Dual_USB_to_UART_Bridge_Controller_011D1A9D-if01-port0" \
  ./run.sh
```

`compose.yaml` already declares `BACKEND_COMMAND` as a pass-through env on the backend service, and `services/backend/entrypoint.sh` runs `bash -c "$BACKEND_COMMAND"` whenever it's set. No file edits needed for the one-off.

---

_It's been a long, successful semester. Good cart._
