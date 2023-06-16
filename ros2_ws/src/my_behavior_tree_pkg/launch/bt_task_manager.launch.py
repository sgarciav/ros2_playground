from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "tree_filename",
            default_value="main_tree.xml",
            description="Name of the xml tree file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "sub_topic",
            default_value="/bt_sub_topic",
            description="Name of the ROS topic that the BT subscriber node is listening to.",
        )
    )

    # Initialize Arguments
    tree_filename = LaunchConfiguration("tree_filename")
    subscriber_topic_name = LaunchConfiguration("sub_topic")

    path_to_tree_file = PathJoinSubstitution(
        [FindPackageShare("my_behavior_tree_pkg"), "behavior_trees", tree_filename]
    )

    behavior_tree_task_manager = Node(
        package="my_behavior_tree_pkg",
        executable="bt_task_manager",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[
            {'path_to_tree': path_to_tree_file},
            {'subscriber_topic_name': subscriber_topic_name},
        ],
    )

    nodes = [
        behavior_tree_task_manager,
    ]

    return LaunchDescription(declared_arguments + nodes)
