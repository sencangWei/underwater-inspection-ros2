import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    own = get_package_share_directory('underwater_inspection')
    sim = get_package_share_directory('stonefish_ros2')
    desc = get_package_share_directory('stonefish_description')
    control = get_package_share_directory('stonefish_control')
    allocator = get_package_share_directory('stonefish_thruster_manager')
    return LaunchDescription([
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('output_dir', default_value='/tmp/underwater-inspection-run'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(sim, 'launch/simulator.launch.py')),
            launch_arguments={'simulation_data': desc,
                'scenario_desc': os.path.join(own, 'scenarios/inspection.scn'),
                'simulation_rate': '100.0', 'window_res_x': '960', 'window_res_y': '720',
                'rendering_quality': 'medium', 'gpu': 'true'}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(allocator, 'launch/thruster_manager.launch.py')),
            launch_arguments={'vehicle_name': 'bluerov2', 'use_sim_time': 'false'}.items()),
        Node(package='stonefish_control', executable='position_controller_node',
             namespace='bluerov2', name='position_controller_4dof', output='screen',
             parameters=[os.path.join(desc, 'data/robots/bluerov2/config/dynamics_params.yaml'),
                         os.path.join(control, 'config/bluerov2/position_controller.yaml'),
                         {'use_sim_time': False}]),
        Node(package='underwater_inspection', executable='mission', output='screen',
             parameters=[{'use_sim_time': False,
                          'config_path': os.path.join(own, 'config/mission.yaml'),
                          'output_dir': LaunchConfiguration('output_dir')}],
             condition=IfCondition(LaunchConfiguration('mission'))),
        Node(package='underwater_inspection', executable='viewer', output='screen',
             condition=IfCondition(LaunchConfiguration('viewer'))),
    ])
