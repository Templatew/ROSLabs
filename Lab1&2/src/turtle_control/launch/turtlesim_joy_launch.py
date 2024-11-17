from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

param_file = os.path.join(
    get_package_share_directory('turtle_control'),
    'config',
    'params.yaml'
)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            package='turtle_control',
            executable='turtle_joy',
            name='turtle_joy'
        ),
])