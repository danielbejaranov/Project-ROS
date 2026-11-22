"""Abre Sakhir con cinco obstáculos reales; coche detenido para inspección."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import sys


def generate_launch_description():
    here = Path(__file__).resolve().parent
    stem = here.parent / 'results/Sakhir_obstacles_5'
    for suffix in ('.yaml', '.png'):
        if not Path(str(stem) + suffix).is_file():
            raise FileNotFoundError('Genera el mapa con scenarios/build_obstacle_map.py primero.')
    share = Path(get_package_share_directory('f1tenth_gym_ros'))
    return LaunchDescription([
        LogInfo(msg='Sakhir: cinco obstáculos de ocupación. Vista de inspección; coche detenido.'),
        Node(package='rviz2', executable='rviz2', name='rviz_obstacles',
             parameters=[{'use_sim_time': True}], arguments=['-d', str(here/'obstacles.rviz')], output='screen'),
        Node(package='f1tenth_gym_ros', executable='gym_bridge', name='bridge',
             parameters=[str(share/'config/sim.yaml'), {'validation_lockstep': False, 'map_path': str(stem)}], output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename':str(stem)+'.yaml', 'use_sim_time':True}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='obstacle_map_lifecycle', parameters=[{'use_sim_time':True, 'autostart':True,
                                                       'node_names':['map_server']}], output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='ego_robot_state_publisher',
             parameters=[{'use_sim_time':True, 'robot_description':Command(['xacro ',str(share/'launch/ego_racecar.xacro')])}],
             remappings=[('/robot_description','/ego_robot_description')], output='screen'),
        ExecuteProcess(cmd=[sys.executable, str(here/'obstacle_markers.py'), str(here/'static_obstacles.yaml')], output='screen'),
    ])
