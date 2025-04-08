import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_diff_robot = get_package_share_directory('diff_robot_gazebo')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    urdf_file = os.path.join(pkg_diff_robot, 'urdf', 'diff_robot.urdf')
    world_file = os.path.join(pkg_diff_robot, 'worlds', 'diff_robot_world.sdf')

    urdf_file_name = 'diff_robot.urdf'
    urdf = os.path.join(
        get_package_share_directory('diff_robot_gazebo'), 'urdf',
        urdf_file_name)

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'diff_robot'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description':  robot_desc}],
            arguments=[urdf]
        ),
    ])
