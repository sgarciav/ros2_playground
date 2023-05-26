FROM osrf/ros:humble-desktop-full

MAINTAINER Sergio Garcia
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
    git \
    openssl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create the "ros" user, add user to sudo group
ARG USER_ID=1000
ARG GROUP_ID=1000

ENV USERNAME ros
RUN adduser --disabled-password --gecos '' $USERNAME \
    && usermod  --uid ${USER_ID} $USERNAME \
    && groupmod --gid ${GROUP_ID} $USERNAME \
    && usermod --shell /bin/bash $USERNAME \
    && adduser $USERNAME sudo \
    && adduser $USERNAME dialout \
    && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER $USERNAME

# Install code dependencies and setup ros environment
RUN sudo apt-get update \
    && rosdep update \
    && source /opt/ros/humble/setup.bash \
    && echo 'source /opt/ros/humble/setup.bash' >> /home/$USERNAME/.bashrc \
    && echo 'source /usr/share/colcon_cd/function/colcon_cd.sh' >> /home/$USERNAME/.bashrc

RUN mkdir -p /home/$USERNAME/workspaces/ros2_ws
WORKDIR /home/$USERNAME/workspaces

# Build the packages in the ROS2 workspace
COPY --chown=ros ros2_ws ros2_ws
RUN source /opt/ros/humble/setup.bash \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=humble \
    && cd ros2_ws \
    && colcon build --symlink-install \
    && echo 'source ~/workspaces/ros2_ws/install/local_setup.bash' >> /home/$USERNAME/.bashrc
