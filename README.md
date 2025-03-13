# docker_files
Dockerfile and scripts for complete setup and operation of a JACart.

# Required Hardware/Software
1. x86_64 unix system
1. graphics card that supports nvidia CUDA
1. Linux Mint 21.3 or Ubuntu 22.04 (other distributions haven't been tested, and installation scripts wouldn't work on them).

# Installation

First clone this repo.

## Required Dependencies

1. [Install Nvidia Cuda Drivers](https://developer.nvidia.com/cuda-downloads?target_os=Linux&target_arch=x86_64&Distribution=Ubuntu&target_version=22.04&target_type=deb_network)
1. [Install Docker Engine](https://docs.docker.com/engine/install/ubuntu/)
1. [Manage Docker as non-root](https://docs.docker.com/engine/install/linux-postinstall/)
1. [Install NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

An attempt was made to write scripts to do this for you (still in the ./require directory). It would be nice if somone could get that working.

# Running
`bash ./run.sh` will run everything for full operation of the cart including initializing the host. This does the steps below for you. This not praticial for development use.

## Initialize Host

`bash ./initialize_host` will 

1. Assign static IP of **192.168.1.254/24** to establish connection with Velodyne Lidar.
1. Allow the docker containers to access the host's X server.

## Docker Compose

`docker compose up` starts the frontend (user interface) and backend (all ros2 nodes & rviz2) services. Refer to [./compose.yaml](./compose.yaml) and ["How Compose works"](https://docs.docker.com/compose/compose-application-model/).

# Development

`bash ./dev-run.sh` will create the containers but override the entry commands so they don't start anything. You should run this before using any of the options below.

## VSCode

Install the [VSCode Devcontainers Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers). This will enable you to code inside the containers. If you create the containers and open the remote explorer tab in VSCode you should see the frontend and backend containers. Here is what it looks like:

![devcontainers](https://github.com/user-attachments/assets/f1954457-1171-4320-a687-2cc7833905c9)

Once you open the container in vscode, opening a terminal (through VSCode) will also give you access to the development container.


## TMUX (maybe to use `nvim` or `nano`)
`docker compose run <backend/frontend> tmux` will run `tmux` in a container. Use `exit` to close it.
'docker exec -it <container id> /bin/bash' will start a bash terminal inside the specified container


# Debugging / Manual Use (for those who want to run it with a little more control)
'docker ps -a / docker ls -a'                     :    List all containers (regardless of status)
'docker images'                                   :    List all image details
'docker build -t <image_name> <dockerfile_path>'  :    Manually rebuild a container
'docker start -ai <container id>'                 :    Manually start a container

# Removal / Manual Clean Up
'docker rm <Container id or name>'  :  Remove specified container
'docker rmi <Image id or name>'     :  Remove specified image

# Notes about manual Building
Manually start the container and connect to a terminal in it using the above commands.
cd to dev_ws/src
'colcon build --symlink-install'
'source install/setup.bash'
Builds it manually -- it is not always built or sourced due to the nature of docker compose
Try this if you feel like your edits aren't affecting the docker.

