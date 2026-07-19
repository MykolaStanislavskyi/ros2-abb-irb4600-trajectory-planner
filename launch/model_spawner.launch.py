from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='block1_stanislavskyi',
            executable='model_spawner',
            name='model_spawner',
            output='screen'
        )
    ])