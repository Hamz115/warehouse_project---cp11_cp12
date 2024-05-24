import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(command, *args, **kwargs):
    map_file = command.launch_configurations['map_file']

    
    map_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file)

    if map_file == 'warehouse_map_real.yaml':
        rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'config', 'real_map.rviz')
        use_sim_time = False
    else:
        rviz_config_dir = os.path.join(get_package_share_directory('map_server'), 'config', 'sim_map.rviz')
        use_sim_time = True

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename':map_path} 
                       ]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-d', rviz_config_dir])
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_real.yaml',
            description='simulation or real robot map.'),
        OpaqueFunction(function=launch_setup)
    ])