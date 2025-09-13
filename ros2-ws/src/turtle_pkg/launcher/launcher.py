from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        Node(
            package='aur_package',
            executable='turtle_chase',
            name='turtle_chase',
            output='screen'
        ),
    ])
