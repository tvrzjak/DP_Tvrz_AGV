import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'Leuze_diff_AGV.urdf'
    urdf = os.path.join(
        get_package_share_directory('Leuze_diff_AGV_description'), 'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    # Retrieve parameters for plc_driver from environment variables
    plc_ip = os.getenv('PLCDRIVER_PLC_IP', '192.168.20.1')  # Default to 192.168.1.100 if not set
    plc_rack = int(os.getenv('PLCDRIVER_PLC_RACK', 0))
    plc_slot = int(os.getenv('PLCDRIVER_PLC_SLOT', 1))
    pub_period = float(os.getenv('PLCDRIVER_PUB_PERIOD', 0.05))
    json_pub_path = os.getenv('PLCDRIVER_JSON_PUB_PATH', '~/ros2_ws/install/plc_driver/share/plc_driver/resources/PLC2ROS.json')
    json_sub_path = os.getenv('PLCDRIVER_JSON_SUB_PATH', '~/ros2_ws/install/plc_driver/share/plc_driver/resources/ROS2PLC.json')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]
        ),

        Node(
            package='Leuze_diff_AGV_description',
            executable='state_publisher',
            name='state_publisher',
            output='screen'
        ),

        Node(
            package='plc_driver',
            executable='start',
            name='start',
            output='screen',
            parameters=[
                {'plc_ip': plc_ip},
                {'plc_rack': plc_rack},
                {'plc_slot': plc_slot},
                {'pub_period': pub_period},
                {'json_pub_path': json_pub_path},
                {'json_sub_path': json_sub_path}
            ]
        ),
    ])
