from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # CSV가 패키지 안에 있을 때 기본 경로 지정
    default_csv = os.path.join(
        get_package_share_directory('ugv_controller'),
        'mission1.csv'
    )

    # Launch arguments
    path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value=default_csv,
        description='Full path to CSV file with waypoints')
    longitudinal_speed_arg = DeclareLaunchArgument(
        'longitudinal_speed',
        default_value='2.0',
        description='Max longitudinal speed for the vehicle')
    arrival_threshold_arg = DeclareLaunchArgument(
            'arrival_threshold',
            default_value='0.5',
            description='Distance threshold to consider waypoint reached')

    # Path follower node
    path_follower_node = Node(
        package='ugv_controller',
        executable='path_follower_node',
        name='path_follower',
        output='screen',
        parameters=[{
            'path_file': LaunchConfiguration('path_file'),
            'longitudinal_speed': LaunchConfiguration('longitudinal_speed'),
            'arrival_threshold': LaunchConfiguration('arrival_threshold')
        }]
    )

    return LaunchDescription([
        path_file_arg,
        longitudinal_speed_arg,
        arrival_threshold_arg,
        path_follower_node
    ])