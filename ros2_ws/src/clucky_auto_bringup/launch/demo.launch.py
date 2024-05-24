from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    auto_server = Node(
        package="actions_cpp",
        executable="navigate_rover_server",
    )

    ld.add_action(auto_server)
    
    return ld