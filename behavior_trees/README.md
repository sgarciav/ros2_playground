# Example: Behavior Trees

[Behavior Trees](https://www.behaviortree.dev/) are your best friend for
replacing your finite state machine.

## First Time: Main .env file

Before spinning up a Docker container, create the main `.env` file:

    cd /path/to/ros2_playground/behavior_trees
    echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env

<!-- ## First Time: 3rd Party Repos -->

<!-- Make sure to have pulled the latest 3rd party repos into the ROS2 workspace -->
<!-- using the `vcs` tool: -->

<!--     cd /path/to/ros2_playground -->
<!--     vcs import ros2_ws/src < tools.repos -->
<!--     vcs pull ros2_ws/src -->

# Run the Example

## Spin the containers

1. Build the Docker image:

        cd /path/to/ros2_playground/behavior_trees
        docker-compose build

2. Spin the container:

        cd /path/to/ros2_playground/behavior_trees
        docker-compose up -d

3. Access the main container:

        docker exec -it ros2_playground_behavior_trees /bin/bash

Once the image is built, you can rerun the tests by starting at Step 2.

**NOTE**: Developers can access any number of containers by opening a new
terminal and executing Step 3. This will drop you into a new spinning container
that will be connected to all other spinning containers on the same machine.

## Run



## Stop the containers

Exit out of all containers, and stop the spinning containers by executing:

    cd /path/to/ros2_playground/behavior_trees
    docker-compose stop

<!-- # Limitations -->

<!-- TODO -->
