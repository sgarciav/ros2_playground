FROM osrf/ros:galactic-desktop-focal

MAINTAINER Sergio Garcia
ENV DEBIAN_FRONTEND noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update \
    && apt-get install -y \
    git \
    openssl \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
    && apt update \
    && apt install -y ros-noetic-desktop-full

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

# Install code dependencies
RUN sudo apt-get update \
    && rosdep update

RUN mkdir -p /home/$USERNAME/workspaces/ros2_ws
WORKDIR /home/$USERNAME/workspaces

# Build the packages in the ROS2 workspace (except the ROS bridge)
COPY --chown=ros ros2_ws ros2_ws
RUN source /opt/ros/galactic/setup.bash \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=galactic \
    && cd ros2_ws \
    && colcon build --symlink-install --packages-skip ros1_bridge

# Build the ROS bridge package
RUN source /opt/ros/noetic/setup.bash \
    && sudo rosdep install --from-paths . --ignore-src -r -y --rosdistro=noetic \
    && source /opt/ros/galactic/setup.bash \
    && cd ros2_ws \
    && colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
