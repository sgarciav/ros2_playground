# Example: ROS1 <> ROS2 Bridge

While working through this example, keep in mind that there are two versions of
ROS installed in the same Docker environment: ROS1 Noetic and ROS2 Galactic. By
default, neither environment is sourced. It'll be up to the developer to source
the corresponding environment depending on the instructions so keep this in mind
while reading through this README.

## First Time: Main .env file

Before spinning up a Docker container, create the main `.env` file:

    cd /path/to/ros2_playground/ros_bridge
    echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env

## First Time: 3rd Party Repos

Make sure to have pulled the latest 3rd party repos into the ROS2 workspace
using the `vcs` tool:

    cd /path/to/ros2_playground
    vcs import ros2_ws/src < tools.repos
    vcs pull ros2_ws/src

# Run the Example

## Spin the containers

1. Build the Docker image:

        cd /path/to/ros2_playground/ros_bridge
        docker-compose build

    Grab some coffee. The image will take some time to build the `ros1_bridge`
    package and no progress will be displayed.

2. Spin the container:

        cd /path/to/ros2_playground/ros_bridge
        docker-compose up -d

3. Access the main container:

        docker exec -it ros2_playground_rosbridge /bin/bash

Once the image is built, you can rerun the tests by starting at Step 2.

**NOTE**: Developers can access any number of containers by opening a new
terminal and executing Step 3. This will drop you into a new spinning container
that will be connected to all other spinning containers on the same machine.

## Run

Each of the following set of commands will be executed inside a new spinning
container - each in ts own terminal:

1. Terminal 1: Start the ROS1 roscore:

        source /opt/ros/noetic/setup.bash
        roscore

2. Terminal 2: Start the ROS1 <> ROS2 bridge:

        source /opt/ros/galactic/setup.bash
        source ~/workspaces/ros2_ws/install/local_setup.bash
        source /opt/ros/noetic/setup.bash
        ros2 run ros1_bridge dynamic_bridge

3. Terminal 3: Run the turtle sim **ROS2** node:

        source /opt/ros/galactic/setup.bash
        source ~/workspaces/ros2_ws/install/local_setup.bash
        ros2 run turtlesim turtlesim_node

4. Terminal 4: Run the **ROS1** teleop node:

        source /opt/ros/noetic/setup.bash
        rosrun turtlesim turtle_teleop_key

At this point you should be able to command the ROS2 turtle from a ROS1 teleop
interaction. The bridge node is taking care of translating the ROS1 messages in
the `/turtle1/cmd_vel` topic to ROS2 messages.

As an experment, you can kill the ros bridge node and see how the
communication between the teleop and turtle nodes doesn't exist anymore.

This example was taken
from [this YouTube video](https://www.youtube.com/watch?v=sJLvv1xtjSM).

## Stop the containers

Exit out of all containers, and stop the spinning containers by executing:

    cd /path/to/ros2_playground/ros_bridge
    docker-compose stop

# Limitations

Not all message types are supported. Current support is for common ROS
interfaces (messages/services), such as the interface packages listed in
the
[ros2/common_interfaces repository](https://github.com/ros2/common_interfaces)
and *tf2_msgs*. If you would like to use a bridge with other interfaces
(including your own custom types), you will have to build the bridge from
source. [These are the instructions](https://github.com/ros2/ros1_bridge/tree/galactic#building-the-bridge-from-source) for
doing so.
