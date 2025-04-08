from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='SW_OGS_ros2_driver',
            executable='leuze_driver_ogs',
            name='leuze_driver_ogs',
            output='screen',
        )
    ])