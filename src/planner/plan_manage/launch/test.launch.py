from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Path to the YAML config file
    config_file = os.path.join(
        get_package_share_directory('ego_planner'),
        'config',
        'config.yaml'
    )

    return LaunchDescription([

        Node(
            package='ego_planner',
            executable='ego_planner_node',
            name='ego_planner_node',
            output='screen',
            parameters=[config_file],
            remappings=[
            ('odom_world', '/drone0/fmu/out/vehicle_odometry')
            ]
        )
    ])