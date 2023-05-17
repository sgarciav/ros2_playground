from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import LaunchConfigurationEquals
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from rif_launch.utils import (load_yaml, get_robot_description_semantic,
                              get_robot_description_kinematics)


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
        [FindPackageShare("my_behavior_tree"), "behavior_trees", tree_filename]
    )


    beahvior_tree_engine_node = Node(
        package="my_behavior_tree",
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
