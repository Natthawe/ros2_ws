from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the parameter reconfigure node
        Node(
            package='reconfig',
            executable='node_reconfig',
            name='parameter_reconfigure_node',
            output='screen',
        ),
        # Launch the GUI reconfigure node
        Node(
            package='reconfig',
            executable='gui_reconfig',
            name='parameter_reconfigure_gui',
            output='screen',
        ),
    ])
