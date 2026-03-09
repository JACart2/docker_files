FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# install nodejs
RUN apt-get update && apt-get install -y wget
RUN wget -O /tmp/nodesource_setup.sh https://deb.nodesource.com/setup_22.x 
RUN bash /tmp/nodesource_setup.sh
RUN apt-get install -y nodejs

# installs for development
RUN apt-get install -y neovim tmux git

# install gh
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg ca-certificates && \
    mkdir -p /etc/apt/keyrings && \
    curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null && \
    chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | tee /etc/apt/sources.list.d/github-cli.list > /dev/null && \
    apt-get update && \
    apt-get install -y gh

# get source code
WORKDIR /root/dev_ws
RUN git clone https://github.com/JACart2/ui.git
WORKDIR /root/dev_ws/ui
RUN npm install

# Get student auth setup script
RUN wget -O /root/student_auth_setup.sh https://raw.githubusercontent.com/JACart2/docker_files/main/student_auth_setup.sh && \
    chmod +x /root/student_auth_setup.sh

# setup entrypoint
COPY ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/bash","/entrypoint.sh"]

# install neovim for development in container
RUN apt-get install -y neovim

