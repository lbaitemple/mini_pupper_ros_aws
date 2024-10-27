from launch import LaunchDescription
import os
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    dance_config_path = os.path.join(
        get_package_share_directory('mini_pupper_dance'),
        'routines'
    )

    bringup_path = os.path.join(
        get_package_share_directory('mini_pupper_bringup'),
        'launch/bringup.launch.py'
    )
    env = SetEnvironmentVariable(
            'PYTHONPATH',
            os.pathsep.join([dance_config_path, os.environ.get('PYTHONPATH', '')])
    )
    dance_server_node = Node(
            package="mini_pupper_dance",
            namespace="",
            executable="service",
            name="dance_server",
        )
    dance_client_node = Node(
            package="mini_pupper_dance",
            namespace="",
            executable="client",
            name="dance_client",
        )
    pose_controller_node = Node(
            package="mini_pupper_dance",
            namespace="",
            executable="pose_controller",
            name="pose_controller",
        )
    music_server_node = Node(
            package="mini_pupper_music",
            namespace="",
            executable="service",
            name="music_server",
            parameters=[
                {"music_config_path": dance_config_path}
           ]
        )
    dance_node = Node(
            package="mini_pupper_dance",
            namespace="",
            executable="dance",
            name="dance",
            parameters=[
                {"dance_config_path": dance_config_path}
            ]
        ) 
        
    bringup_node=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_path)
    )
    return LaunchDescription([
        dance_node,
        env,
        bringup_node, 
        music_server_node,
        pose_controller_node
    ])
