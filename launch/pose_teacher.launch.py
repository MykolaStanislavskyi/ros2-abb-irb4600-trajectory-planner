from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('abb_model'),
            'urdf',
            'abb_irb4600_60_205.xacro'
        ])
    ])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

        Node(
            package='block1_stanislavskyi',
            executable='model_spawner',
            name='model_spawner',
            output='screen',
        ),

        Node(
            package='block1_stanislavskyi',
            executable='pose_teacher',
            name='pose_teacher',
            output='screen',
        ),
    ])