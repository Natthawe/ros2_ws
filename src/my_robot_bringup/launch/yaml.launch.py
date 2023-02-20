import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('my_robot_bringup'),
        'config',
        'params.yaml'
    )

    yaml_node = Node(
        package="my_python_pkg",
        executable="params",
        name="Params_Node",
        namespace="namespace",
        parameters= [config]
    )

    ld.add_action(yaml_node)
    return ld