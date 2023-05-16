# About

This repo hosts environments so developers can play with some ROS 2 tools. All
the examples are run in Docker environments. The examples hosted in this repo
are meant to serve as a starting point for getting familiar with ROS 2 tools. It
is **not** intended as a comprehensive guide. Developers are encouraged to look
through the corresponding documentation to learn more about each topic.

Examples hosted in this repo include:
* ROS 1 <> ROS 2 bridge
* Husarnet VPN solution
* ROS 2 Actions

# First Time Instructions

Follow these instructions first regardless of the example you want to play
with.

## Install Dependencies

* [Docker](https://docs.docker.com/engine/install/ubuntu/)
* [docker-compose](https://docs.docker.com/compose/install/)
* [VCS](http://wiki.ros.org/vcstool): `sudo apt install python3-vcstool`

## Clone this repo

Clone this repo to a location of your choosing:

    git clone git@github.com:sgarciav/ros2_playground.git

Note that some examples depend on other 3rd party repositories. Their
corresponding README files will inform when to run the vcs command to import
these repos into the provided ROS 2 workspace.
