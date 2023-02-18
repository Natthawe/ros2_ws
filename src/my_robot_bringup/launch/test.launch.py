from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="my_talker",
        remappings=[
            ("chatter", "my_chatter") #tuple
        ],
        parameters= [
            {"param_name": "value"}, #dictionaries
            {"param_id": 1}
        ]

    )

    listener_node = Node(
        package="demo_nodes_py",
        executable="listener",
        name="my_listener",
        remappings=[
            ("chatter", "my_chatter")
        ]
    )

    ld.add_action(talker_node)
    ld.add_action(listener_node)

    return ld