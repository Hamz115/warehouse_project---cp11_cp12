import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(command, *args, **kwargs):
    map_file = command.launch_configurations['map_file']

    # Default values for map and RViz config directories
    map_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file)

    if map_file == 'warehouse_map_real.yaml':
        rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config', 'real_local.rviz')
        localization_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'real_amcl_config.yaml')
        service_server = 'approach_service_server'
        execute_process = []
        use_sim_time = False
    else:
        rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config', 'sim_local.rviz')
        localization_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
        service_server = 'sim_approach_service_server'
        execute_process = []
        use_sim_time = True

    return [
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, 
                        {'yaml_filename': map_path}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[localization_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]),

        Node(
            package='attach_service',
            executable= service_server,
            name= service_server,
            output='screen'),
        
        *execute_process

    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_real.yaml',
            description='Type of map to use.'),
        OpaqueFunction(function=launch_setup)
    ])

# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
# from launch.substitutions import LaunchConfiguration
# from launch_ros.actions import Node

# def launch_setup(context, *args, **kwargs):
#     map_file = context.launch_configurations['map_file']

#     # Default values for map and RViz config directories
#     map_path = os.path.join(get_package_share_directory('map_server'), 'config', map_file)

#     if map_file == 'warehouse_map_real.yaml':
#         rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config', 'real_local.rviz')
#         localization_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config.yaml')
#         server_excecutable = 'approach_service_server'
#         use_sim_time = False
#     else:
#         rviz_config_dir = os.path.join(get_package_share_directory('localization_server'), 'config', 'sim_local.rviz')
#         localization_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'sim_amcl_config.yaml')
#         server_excecutable = 'sim_approach_service_server'
#         use_sim_time = True

#     return [
#         Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time}, 
#                         {'yaml_filename': map_path}]),

#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[localization_yaml]),

#         Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_mapper',
#             output='screen',
#             parameters=[{'use_sim_time': use_sim_time},
#                         {'autostart': True},
#                         {'node_names': ['map_server', 'amcl']}]),
        
#         Node(
#             package='attach_service',
#             executable=server_excecutable,
#             name=server_excecutable,
#             output='screen'),

#         ExecuteProcess(
#             cmd=['ros2', 'service', 'call', 
#                  '/reinitialize_global_localization', 'std_srvs/srv/Empty'],         
#             output='screen')
#     ]

# def generate_launch_description():
#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'map_file',
#             default_value='warehouse_map_real.yaml',
#             description='Type of map to use.'),
#         OpaqueFunction(function=launch_setup)
#     ])
    