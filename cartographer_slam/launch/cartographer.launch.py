import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(command, *args, **kwargs):
    mode = command.launch_configurations['mode']

    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')

    if mode == 'real':
        configuration_basename = 'realcartographer.lua'
        rviz_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config', 'real_cartographer.rviz')
        use_sim_time = False
    else:
        configuration_basename = 'cartographer.lua'
        rviz_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config', 'sim_cartographer.rviz')
        use_sim_time = True

    return [
        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename]),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            output='screen',
            name='occupancy_grid_node',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

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
            'mode',
            default_value='real',
            description='Simulation or real mode.'),
        OpaqueFunction(function=launch_setup)
    ])