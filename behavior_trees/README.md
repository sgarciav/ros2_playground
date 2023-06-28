# Example: Behavior Trees

[Behavior Trees](https://www.behaviortree.dev/) (BTs) are your best friend for
implementing your finite state machine. Essentially, it's a lovely interface
that wraps all your ROS Actions and provides an easy way to organize them as
needed. This allows for a robust and modular implementation of the behaviors of
interest.

[This article](https://medium.com/@nullbyte.in/behavior-trees-for-ros2-part-1-unlocking-advanced-robotic-decision-making-and-control-7856582fb812) provides
a good explanation of what are behavior trees and how the software works. At
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

3. Nodes can also inherit from the `BT::RosTopicSubNode` or
   `BT::RosTopicPubNode` classes. Their function is to subscribe or publish to
   ROS topics, respectively. Use these nodes to pass data between ROS and the BT
   Blackboard.

4. There needs to be a task manager ROS 2 node that registers all the BT nodes,
   loads the tree, and executes the tree. The `bt_task_manager` ROS 2 node in
   this example provides all the basics needed.

## Examples Examplanation

This package provides working examples for some basic concepts related to BTs
and their interactions with ROS 2. Namely:

* BT nodes that inherit from the `SyncActionNode` class.
  * Example:
    [SaySomething](https://github.com/sgarciav/ros2_playground/blob/master/ros2_ws/src/my_behavior_tree_pkg/src/saysomething_btnode.cpp)
  * Simple BT nodes that do not have a relationship with ROS.
  * They are controlled by the `onTick()` function, which defines their
    behavior.

* BT nodes that inherit from the `RosActionNode` class.
  * Example:
    [Fibonacci](https://github.com/sgarciav/ros2_playground/blob/master/ros2_ws/src/my_behavior_tree_pkg/src/fibonacci_btnode.cpp)
  * BT nodes that work as the ROS 2 action clients for their corresponding
    action servers.
  * Their function is to send a request to their action server, which is
    reponsible for actually carrying out the task, and then return the status of
    the action.

* BT nodes that inherit from the `RosTopicSubNode` class.
  * Example:
    [RosToBlackboard](https://github.com/sgarciav/ros2_playground/blob/master/ros2_ws/src/my_behavior_tree_pkg/include/my_behavior_tree_pkg/ros_to_blackboard_btnode.hpp)
  * BT nodes that subscribe to a ROS topic and, typically, write the value to
    the Blackboard for the rest of the BT architecture to leverage.

* BT nodes that inherit from the `RosTopicPubNode` class.
  * Example:
    [BlackboardToRos](https://github.com/sgarciav/ros2_playground/blob/master/ros2_ws/src/my_behavior_tree_pkg/include/my_behavior_tree_pkg/blackboard_to_ros_btnode.hpp)
  * BT nodes that read a value from the Blackboard and, typically, publish the
    value to a ROS topic.

* Input/output ports that handle non-string message types (e.g., ROS messages).
  * Example:
    [CalculateGoal](https://github.com/sgarciav/ros2_playground/blob/master/ros2_ws/src/my_behavior_tree_pkg/src/calculategoal_btnode.cpp) to
    write to the Blackboard,
    and
    [PrintTarget](https://github.com/sgarciav/ros2_playground/blob/master/ros2_ws/src/my_behavior_tree_pkg/src/printtarget_btnode.cpp) to
    read from the Backboard.

## Special Nodes

The `RosToBlackboard` and `BlackboardToRos` nodes are special because they're
templated classes - meaning that they work with any type of ROS message without
requiring developers to define and compile a new class for each ROS message type
of interest.

Developers can register a node to the tree factory by specifying ANY ROS message
type. They can register it as many times as they want but MAKE SURE that the
node name and the topic it's susbcribing to are UNIQUE. The name is how you
reference it in the xml tree.

A typical notation is `RosToBlackboard[MESSAGE]`, where `[MESSAGE]` is the ROS
message type. Same for the `BlackboardToRos` node.

# First Time Instructions

## Main .env file

Before spinning up a Docker container, create the main `.env` file:

    cd /path/to/ros2_playground/behavior_trees
    echo -e "USER_ID=$(id -u ${USER})\nGROUP_ID=$(id -g ${USER})" > .env

## 3rd Party Repos

Make sure to have pulled the latest 3rd party repos into the ROS2 workspace
using the `vcs` tool:

    cd /path/to/ros2_playground/behavior_trees
    vcs import ../ros2_ws/src < tools.repos
    vcs pull ../ros2_ws/src

## Shared Directory (optional)

The `docker-compose` file for this project allows for a shared directory between
the host and the Docker container. If you'd like to make use of it, create the
shared directory in your host (instructions below) and the compose file will do
the rest. This is helpful to add installer file like Groot2, for example.

    cd /path/to/ros2_playground
    mkdir -p shared_dir

**NOTE**: These instructions **MUST** be executed **BEFORE** you sping up the
container.

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

1. In terminal 1, run the ROS 2 action server:

        ros2 run my_action_pkg fibonacci_server

2. In terminal 2, run the behavior tree task manager to start the tree:

        ros2 launch my_behavior_tree_pkg bt_task_manager.launch.py tree_filename:=main_tree_groot.xml

    The tree will start and you will see the following message displayed:

    ```
    [bt_task_manager-1] ===========
    [bt_task_manager-1] Robot says: Hello, World!
    [bt_task_manager-1] ===========
    ```

    Afterwards, the tree will pause at the `RosToBlackboard` task and wait idly.

3. In terminal 3, publish a message to the corresponding fibonacci topic:

        ros2 topic pub -r 3 /ros_to_blackboard/int std_msgs/msg/Int32 "{data: 9}"

    At this point you should see the behavior tree manager go through the tasks
    specified in the selected xml tree.

## Stop the containers

Exit out of all containers, and stop the spinning containers by executing:

    cd /path/to/ros2_playground/behavior_trees
    docker-compose stop
