# Example: Husarnet

## First Time: Join Code

First, you'll need to setup an account on [husarnet.com](husarnet.com) and
create a network with its corresponding join code. Instructions on setting up a
network and obtaining the join code
are [here](https://husarnet.com/docs#using-husarnet).

Copy the join code and update the `husarnet-config` files inside the `field-env`
and `home-env` directories. The `JOINCODE` is the same in both files, only the
`HOSTNAME` parameter is different. No need to modify this.

## First Time: Main .env file

Before spinning up a Docker container, create the main `.env` file to let the
Docker environment know which side of the network it's spinning:

    cd /path/to/ros2_playground/husarnet
    echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env
    echo -e "HUSARNET_SIDE=xxx" >> .env

where `xxx` is `field` (setup on the system at the field) or `home` (setup on
the home base machines).

# Run the Example

## Spin the containers

You'll need to follow these instructions twice: once on the field machine and
once on the home machine:

1. Build the Docker image:

        cd /path/to/ros2_playground/husarnet
        docker-compose build

2. Spin the containers:

        cd /path/to/ros2_playground/husarnet
        docker-compose up -d

3. Access the main container:

        docker exec -it ros2_playground_husarnet /bin/bash

Once the image is built, you can rerun the tests by starting at Step 2.

## Run

**On the home machine**: start the sim node:

        ros2 run turtlesim turtlesim_node

**On the field machine**: run the controller:

        ros2 launch my_turtle_bringup only_controller.launch.py

At this point you should see, on the home machine, the ROS turtle spin around
while the trajectory changes colors.

## Stop the containers

Exit from the containers, and stop the spinning containers by executing:

    cd /path/to/ros2_playground/husarnet
    docker-compose stop

# Limitations

This current implementation does not allow for multiple containers spinning on
the same machine. This is because the `network_mode` parameter is liminted to
the husarnet service. Currently looking for a solution such that developers can
define the ntework mode both to the husarnet service and `host`.
