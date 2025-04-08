from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Line_Follower_PD_controller',
            executable='line_follower_pd',
            name='line_follower',
            output='screen',
        )
    ])