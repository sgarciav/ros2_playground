# About

This repo hosts environments so developers can play with some ROS2 tools. All
the examples are run in Docker environments. The examples hosted in this repo
are meant to serve as a starting point for getting familiar with ROS2 tools. It
is **not** intended as a comprehensive guide. Developers are encouraged to look
through the corresponding documentation to learn more about each topic.

Examples hosted in this repo include:
* ROS1 <> ROS2 bridge ([link](https://github.com/Greenzie/ros2_test_environment/tree/master/ros_bridge))
* Husarnet VPN solution ([link](https://github.com/Greenzie/ros2_test_environment/tree/master/husarnet))

# First Time Instructions

Follow these instructions first regardless of the example you want to play
with.

## Install Dependencies

* [Docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [VCS](http://wiki.ros.org/vcstool): `sudo apt install python3-vcstool`

## Setup Workspace

1. Setup workspace and clone this repo:

        mkdir -p ~/ros2/playground_ws/data
        mkdir -p ~/ros2/playground_ws/src
        cd ~/ros2/playground_ws
        git clone git@github.com:sgarciav/ros2_playground_setup.git

2. Clone repositories:

        cd /path/to/ros2_playground_setup
        vcs import ../src < tools.repos