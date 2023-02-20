from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    yaml_node = Node(
        package="my_python_pkg",
        executable="params",
    )

    ld.add_action(yaml_node)
    return ld