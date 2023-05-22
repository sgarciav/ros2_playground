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

    # Initialize Arguments
    tree_filename = LaunchConfiguration("tree_filename")

    path_to_tree_file = PathJoinSubstitution(
        [FindPackageShare("my_behavior_tree_pkg"), "behavior_trees", tree_filename]
    )


    beahvior_tree_engine_node = Node(
        package="my_behavior_tree_pkg",
        executable="behavior_tree_engine",
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        parameters=[
            {'path_to_tree': path_to_tree_file},
        ],
    )

    nodes = [
        beahvior_tree_engine_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
