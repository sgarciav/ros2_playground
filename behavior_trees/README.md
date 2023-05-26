# Example: Behavior Trees

[Behavior Trees](https://www.behaviortree.dev/) are your best friend for
implementing your finite state machine. Essentially, it's a lovely interface
that wraps all your ROS Actions and provides an easy way to organize them as
needed. This allows for a robust and modular implementation of the behaviors of
interest.

[This article](https://medium.com/@nullbyte.in/behavior-trees-for-ros2-part-1-unlocking-advanced-robotic-decision-making-and-control-7856582fb812) provides
a good explanation of what are behavior trees and how teh software works. At
the time of writing this README, the author has not yet released a post about
how to integrate behavior trees with ROS 2. This repo provides a working example
on how to do so.

To following are notes to summarize the general architecture:

1. The ROS 2 action server is a typical server node. This example leverages the
   Fibonacci action server defined in the `my_action_pkg` package.

2. Instead of implementing a ROS 2 action client node, you'll need a BT node
   that inherits from the `BT::RosActionNode` class. Ideally, each node is its
   own static library. The functions defined in this class will interact with
   its corresponding action server while it's running. The client used in this
   example was
   taken [from here](https://www.behaviortree.dev/docs/ros2_integration/).

3. There needs to be a task manager ROS 2 node that registers all the BT nodes,
   loads the tree, and executes the tree. The `bt_task_manager` ROS 2 node in
   this example provides all the basics needed.

## First Time: Main .env file

Before spinning up a Docker container, create the main `.env` file:

    cd /path/to/ros2_playground/behavior_trees
    echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env

## First Time: 3rd Party Repos

Make sure to have pulled the latest 3rd party repos into the ROS2 workspace
using the `vcs` tool:

    cd /path/to/ros2_playground/behavior_trees
    vcs import ../ros2_ws/src < tools.repos
    vcs pull ../ros2_ws/src

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

Once the image is built, you can spin up the container by starting at Step 2.

**NOTE**: Developers can access any number of containers by opening a new
terminal and executing Step 3. This will drop you into a new spinning container
that will be connected to all other spinning containers on the same machine.

## Run

1. In one termina, run the ROS 2 action server:

        ros2 run my_action_pkg fibonacci_server

2. In a different terminal, run the behavior tree task manager:

        ros2 launch my_behavior_tree_pkg bt_task_manager.launch.py

At this point you should see the behavior tree manager go through the two tasks
specified in the xml tree: 1) SaySomething, and 2) FibonacciAction.

## Stop the containers

Exit out of all containers, and stop the spinning containers by executing:

    cd /path/to/ros2_playground/behavior_trees
    docker-compose stop
