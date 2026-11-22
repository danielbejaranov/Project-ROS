"""Interactive native ROS 2 demonstration of two independent LiDAR controllers.

Launch with ./start_ros2.sh ros2 launch scenarios/dynamic_demo.launch.py.
A new timestamped evidence directory is created on every launch. Both cars run
until each has at least ten full laps, preserving all extra laps, then brake.
Close the launch with Ctrl+C after the observer reports completion.
"""
from datetime import datetime, timezone
import os
from pathlib import Path
import sys
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    here=Path(__file__).resolve().parent;project=here.parent
    sys.path.insert(0,str(here))
    from run_dynamic import configure_views
    map_stem=Path(os.environ.get('DANIEL_F1TENTH_MAP',str(project/'results/Sakhir_map'))).resolve()
    for suffix in ('.png','.yaml'):
        if not map_stem.with_suffix(suffix).is_file():raise FileNotFoundError(map_stem.with_suffix(suffix))
    out=project/'scenarios'/'runs'/('dynamic_demo_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ'))
    (out/'run').mkdir(parents=True,exist_ok=False)
    config=yaml.safe_load((here/'dynamic_controllers.yaml').read_text())
    share=Path(get_package_share_directory('f1tenth_gym_ros'))
    bridge=yaml.safe_load((share/'config/sim.yaml').read_text());params=bridge['bridge']['ros__parameters']
    params.update(num_agent=2,map_path=str(map_stem),validation_lockstep=False)
    for suffix,name in [('', 'ego'),('1','opp')]:
        x,y,yaw=config[name]['start'];params.update({f'sx{suffix}':x,f'sy{suffix}':y,f'stheta{suffix}':yaw})
    bridge_config=out/'dynamic_bridge.yaml';bridge_config.write_text(yaml.safe_dump(bridge,sort_keys=False))
    overview,_=configure_views(out)
    nodes=[
        LogInfo(msg=f'Two-vehicle evidence: {out}; both >=10 laps, all extras preserved. Ctrl+C closes this launch.'),
        ExecuteProcess(cmd=[sys.executable,str(here/'dynamic_monitor.py'),'--out',str(out/'run'),'--timeout','2400'],output='screen'),
        Node(package='f1tenth_gym_ros',executable='gym_bridge',name='bridge',parameters=[str(bridge_config)],output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{'yaml_filename':str(map_stem)+'.yaml','use_sim_time':True}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',parameters=[{'use_sim_time':True,'autostart':True,'node_names':['map_server']}],output='screen'),
        Node(package='rviz2',executable='rviz2',name='rviz_dynamic',parameters=[{'use_sim_time':True}],arguments=['-d',str(overview)],output='screen'),
        ExecuteProcess(cmd=[sys.executable,str(here/'dynamic_controller.py'),'--config',str(here/'dynamic_controllers.yaml'),'--out',str(out/'run'),'--ros-args','-p','use_sim_time:=true'],output='screen'),
    ]
    for name in ('ego','opp'):
        nodes.append(Node(package='robot_state_publisher',executable='robot_state_publisher',name=f'{name}_robot_state_publisher',parameters=[{'use_sim_time':True,'robot_description':Command(['xacro ',str(share/f'launch/{name}_racecar.xacro')])}],remappings=[('/robot_description',f'{name}_robot_description')],output='screen'))
    return LaunchDescription(nodes)
