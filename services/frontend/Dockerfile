FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# install nodejs
RUN apt-get update && apt-get install -y wget
RUN wget -O /tmp/nodesource_setup.sh https://deb.nodesource.com/setup_22.x 
RUN bash /tmp/nodesource_setup.sh
RUN apt-get install -y nodejs

# installs for development
RUN apt-get install -y neovim tmux git

# get source code
WORKDIR /
RUN git clone https://github.com/JACart2/ui.git
WORKDIR /ui
RUN git pull
RUN npm install

# setup entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/bash","/entrypoint.sh"]

# install neovim for development in container
RUN apt-get install -y neovim

