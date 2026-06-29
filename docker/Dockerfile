# -------- Stage 1: Base Build --------
FROM osrf/ros:humble-desktop-full AS base

# Setup environment
ENV TERM linux
ENV DEBIAN_FRONTEND noninteractive
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Copy requirement files and install dependencies
COPY docker/requirements.txt .
RUN apt-get update && apt-get install --no-install-recommends -y $(cat requirements.txt) && rm -rf /var/lib/apt/lists/*
RUN rm requirements.txt

# Add vscode user with same UID and GID as your host system
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && mkdir /home/${USERNAME}/.config \
    && chown ${USER_UID}:${USER_GID} /home/${USERNAME}/.config \
    && apt-get update \
    && apt-get install -y \
        sudo \
        tmux \
        tree \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Switch from root to user
USER $USERNAME

# Create workspace directory
RUN mkdir -p /home/${USERNAME}/ros2_ws/src \
    && chown -R ${USER_UID}:${USER_GID} /home/${USERNAME}/ros2_ws

# Adds to bashrc the ros humble overlay sourcing.
RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc
# Adds colcon autocomplete
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/${USERNAME}/.bashrc

WORKDIR /home/${USERNAME}/ros2_ws

# Add the entrypoint script
COPY ./docker/entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

# Set ENTRYPOINT and CMD
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "/usr/bin/bash" ]
