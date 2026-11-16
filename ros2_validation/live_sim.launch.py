"""Demostración a ritmo de pared con RViz y resultados en una carpeta nueva."""
from datetime import datetime, timezone
from pathlib import Path
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share=Path(get_package_share_directory('f1tenth_gym_ros'))
    config=share/'config/sim.yaml';params=yaml.safe_load(config.read_text())['bridge']['ros__parameters']
    here=Path(__file__).resolve().parent
    map_stem=Path(os.environ.get('DANIEL_F1TENTH_MAP', str(here.parent/'results/Sakhir_map'))).expanduser().resolve()
    for suffix in ('.yaml', '.png'):
        if not map_stem.with_suffix(suffix).is_file():
            raise FileNotFoundError(f'Falta {map_stem}{suffix}. Ejecuta installation/import_sakhir_map.py --zip RUTA_ZIP')
    timestamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ')
    demo=here/'demo'/timestamp
    demo.mkdir(parents=True,exist_ok=False)
    controller_config=here.parent/'config/fast_controller.yaml'
    controller_params=([str(controller_config)] if controller_config.exists() else [])
    controller_params.append({'use_sim_time':True,'csv_path':str(demo/'laps.csv')})
    return LaunchDescription([
        LogInfo(msg=f'Corrida interactiva; CSV de vueltas: {demo / "laps.csv"}'),
        Node(package='rviz2',executable='rviz2',name='rviz',parameters=[{'use_sim_time':True}],arguments=['-d',str(share/'launch/gym_bridge.rviz')],output='screen'),
        Node(package='f1tenth_gym_ros',executable='gym_bridge',name='bridge',parameters=[str(config),{'validation_lockstep':False,'map_path':str(map_stem)}],output='screen'),
        Node(package='nav2_map_server',executable='map_server',parameters=[{'yaml_filename':str(map_stem)+'.yaml','use_sim_time':True}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server']}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='ego_robot_state_publisher',parameters=[{'robot_description':Command(['xacro ',str(share/'launch/ego_racecar.xacro')])}],remappings=[('/robot_description','ego_robot_description')],output='screen'),
        Node(package='daniel_gap',executable='gap_node',parameters=controller_params,output='screen'),
        ExecuteProcess(cmd=['python3',str(here/'live_status.py')],output='screen')
    ])
