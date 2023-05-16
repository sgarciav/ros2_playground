# Example: ROS 2 Actions

Actions are a very important component in ROS applications. You can read more
details about Actions and how to implement them from
the
[ROS docs](https://docs.ros.org/en/humble/Tutorials/Intermediate/Creating-an-Action.html). This
example is the implementation of the Fibonnaci tutorial found on the same link.

## First Time: Main .env file

Before spinning up a Docker container, create the main `.env` file:

    cd /path/to/ros2_playground/ros_actions
    echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env

# Run the Example

## Spin the containers

1. Build the Docker image:

        cd /path/to/ros2_playground/ros_actions
        docker-compose build

2. Spin the container:

        cd /path/to/ros2_playground/ros_actions
        docker-compose up -d

3. Access the main container:

        docker exec -it ros2_playground_rosactions /bin/bash

Once the image is built, you can rerun the tests by starting at Step 2.

**NOTE**: Developers can access any number of containers by opening a new
terminal and executing Step 3. This will drop you into a new spinning container
that will be connected to all other spinning containers on the same machine.

<!-- ## Run -->

<!-- TODO -->

## Stop the containers

Exit out of all containers, and stop the spinning containers by executing:

    cd /path/to/ros2_playground/ros_actions
    docker-compose stop

<!-- # Limitations -->
