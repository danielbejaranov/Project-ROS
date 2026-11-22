"""Run the original-map, two-controller ROS 2 experiment and record real RViz.

Lockstep still uses the unchanged 0.01 s F1TENTH physical step. It waits for
both ROS drive messages before each step; its video is faster than simulation
time. Use --mode wall for a normal-duration demonstration.
"""
import argparse
from datetime import datetime, timezone
import importlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import yaml

PROJECT=Path(__file__).resolve().parents[1]
HERE=Path(__file__).resolve().parent
sys.path.insert(0,str(PROJECT/'ros2_validation'))
from run_real_capture import Run,digest

def launch_text(config,share,map_yaml,overview,follow):
    return f'''from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    return LaunchDescription([
        Node(package='f1tenth_gym_ros',executable='gym_bridge',name='bridge',parameters=[{str(config)!r}],output='screen'),
        Node(package='nav2_map_server',executable='map_server',name='map_server',parameters=[{{'yaml_filename':{str(map_yaml)!r},'topic':'map','frame_id':'map','use_sim_time':True}}],output='screen'),
        Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='lifecycle_manager_localization',parameters=[{{'use_sim_time':True,'autostart':True,'node_names':['map_server']}}],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='ego_robot_state_publisher',parameters=[{{'use_sim_time':True,'robot_description':Command(['xacro ',{str(share/'launch/ego_racecar.xacro')!r}])}}],remappings=[('/robot_description','ego_robot_description')],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',name='opp_robot_state_publisher',parameters=[{{'use_sim_time':True,'robot_description':Command(['xacro ',{str(share/'launch/opp_racecar.xacro')!r}])}}],remappings=[('/robot_description','opp_robot_description')],output='screen'),
        Node(package='rviz2',executable='rviz2',name='rviz_dynamic_overview',parameters=[{{'use_sim_time':True}}],arguments=['-d',{str(overview)!r}],output='screen'),
        Node(package='rviz2',executable='rviz2',name='rviz_dynamic_follow',parameters=[{{'use_sim_time':True}}],arguments=['-d',{str(follow)!r}],output='screen'),
    ])
'''

def configure_views(out):
    base=yaml.safe_load((PROJECT/'ros2_validation/overview.rviz').read_text())
    displays=base['Visualization Manager']['Displays']
    displays=[d for d in displays if d['Class'] not in ('rviz_default_plugins/Path','rviz_default_plugins/Marker')]
    ego=next(d for d in displays if d['Class']=='rviz_default_plugins/RobotModel')
    opp=json.loads(json.dumps(ego));opp['Name']='Opponent real vehicle';opp.pop('Links',None)
    opp['Description Topic']['Value']='/opp_robot_description';displays.append(opp)
    # Each RobotModel is an unchanged simulator TF/URDF, never a drawn animation.
    displays.append({'Class':'rviz_default_plugins/Marker','Name':'Both lap counters, timers and overtakes','Enabled':True,'Value':True,'Namespaces':{},'Topic':{'Value':'/daniel_dynamic/overlay','Depth':5,'Reliability Policy':'Reliable','Durability Policy':'Volatile','History Policy':'Keep Last'}})
    for name,color in [('ego','0; 200; 230'),('opp','255; 80; 50')]:
        displays.append({'Class':'rviz_default_plugins/Path','Name':f'{name.upper()} measured ROS path','Enabled':True,'Value':True,'Topic':{'Value':f'/daniel_dynamic/{name}/path','Depth':5,'Reliability Policy':'Reliable','Durability Policy':'Volatile','History Policy':'Keep Last'},'Line Style':'Lines','Line Width':.05,'Color':color,'Alpha':1.,'Buffer Length':1})
    base['Visualization Manager']['Displays']=displays
    base['Window Geometry'].update({'Width':1080,'Height':1080,'X':0,'Y':0,'Hide Left Dock':True,'Hide Right Dock':True})
    overview=out/'dynamic_overview.rviz';overview.write_text(yaml.safe_dump(base,sort_keys=False))
    follow=json.loads(json.dumps(base))
    follow['Visualization Manager']['Views']['Current'].update({'Target Frame':'ego_racecar/base_link','X':0.,'Y':0.,'Scale':65.,'Angle':0.})
    follow['Window Geometry'].update({'Width':840,'Height':1080,'X':1080,'Y':0})
    path=out/'dynamic_follow.rviz';path.write_text(yaml.safe_dump(follow,sort_keys=False))
    return overview,path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--out',type=Path,required=True);p.add_argument('--config',type=Path,default=HERE/'dynamic_controllers.yaml')
    p.add_argument('--map-stem',type=Path,default=PROJECT/'results/Sakhir_map')
    p.add_argument('--mode',choices=['lockstep','wall'],default='lockstep')
    p.add_argument('--display',default=':190');p.add_argument('--domain',type=int,default=190)
    p.add_argument('--timeout-wall',type=float,default=1800.);p.add_argument('--record-video',action='store_true')
    p.add_argument('--startup-seconds',type=float,default=6.)
    a=p.parse_args()
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    if not Path(rclpy.__file__).resolve().is_relative_to(Path('/opt/ros/humble')):
        raise RuntimeError('Native /opt/ros/humble is required')
    if not a.display.startswith(':') or not a.display[1:].isdigit():raise ValueError('Use a private X display :N')
    if not 0<=a.domain<=232:raise ValueError('ROS domain outside 0..232')
    display_num=a.display[1:]
    if Path(f'/tmp/.X11-unix/X{display_num}').exists() or Path(f'/tmp/.X{display_num}-lock').exists():raise RuntimeError('Private display is occupied')
    out=a.out.resolve();out.parent.mkdir(parents=True,exist_ok=True);out.mkdir(exist_ok=False)
    for name in ('logs','run','ros_logs'):(out/name).mkdir()
    cfg=yaml.safe_load(a.config.read_text());(out/'dynamic_controllers.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    share=Path(get_package_share_directory('f1tenth_gym_ros'))
    bridge=yaml.safe_load((share/'config/sim.yaml').read_text())
    b=bridge['bridge']['ros__parameters'];b.update({'num_agent':2,'map_path':str(a.map_stem.resolve()),'validation_lockstep':a.mode=='lockstep'})
    for prefix,name in [('', 'ego'),('1','opp')]:
        x,y,yaw=cfg[name]['start'];b.update({f'sx{prefix}':x,f'sy{prefix}':y,f'stheta{prefix}':yaw})
    map_yaml=Path(b['map_path']+'.yaml');map_image=Path(b['map_path']+b['map_img_ext'])
    for f in (map_yaml,map_image,HERE/'dynamic_monitor.py',HERE/'dynamic_controller.py'):
        if not f.is_file():raise FileNotFoundError(f)
    bridge_config=out/'dynamic_bridge.yaml';bridge_config.write_text(yaml.safe_dump(bridge,sort_keys=False))
    overview,follow=configure_views(out)
    launch=out/'dynamic_infrastructure.launch.py';launch.write_text(launch_text(bridge_config,share,map_yaml,overview,follow))
    xvfb=shutil.which('Xvfb')
    if xvfb is None:
        xvfb=str(PROJECT.parents[1]/'work/ros_root/usr/bin/Xvfb')
        if not Path(xvfb).is_file():raise RuntimeError('Install Xvfb for a private RViz display')
    env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(a.domain),ROS_LOCALHOST_ONLY='1',ROS_LOG_DIR=str(out/'ros_logs'),DISPLAY=a.display,LIBGL_ALWAYS_SOFTWARE='1',QT_QPA_PLATFORM='xcb',PYTHONUNBUFFERED='1')
    env.setdefault('RMW_IMPLEMENTATION','rmw_fastrtps_cpp');env.setdefault('NUMBA_CACHE_DIR','/tmp/daniel_dynamic_numba')
    Path(env['NUMBA_CACHE_DIR']).mkdir(parents=True,exist_ok=True)
    sources={}
    for mod in ('rclpy','daniel_gap.core','daniel_gap.fast_core','f110_gym.envs.f110_env','f1tenth_gym_ros.gym_bridge'):
        source=Path(importlib.import_module(mod).__file__).resolve();sources[mod]={'path':str(source),'sha256':digest(source)}
    for f in ('dynamic_controller.py','dynamic_monitor.py','dynamic_metrics.py','run_dynamic.py'):
        source=HERE/f;sources[f]={'path':str(source),'sha256':digest(source)}
    manifest={'started_utc':datetime.now(timezone.utc).isoformat(),'mode':a.mode,'domain':a.domain,'display':a.display,'native_python':sys.executable,'native_modules':sources,'map_yaml':str(map_yaml),'map_yaml_sha256':digest(map_yaml),'map_image':str(map_image),'map_image_sha256':digest(map_image),'physics_overrides':{},'controller_config':cfg,'finish_protocol':'at least ten laps EACH; retain all extra laps; simultaneous guided stop; four measured stopped simulation seconds','video_requested':a.record_video,'video_source':'actual RViz X11 framebuffer','timing_note':'Lap times use simulation time. Lockstep video can be faster than this clock.','status':'starting'}
    def save():(out/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    save();run=Run(out,env);started=time.monotonic();error=None;completed=False
    try:
        run.start('xvfb',[xvfb,a.display,'-screen','0','1920x1080x24','-ac','-nolisten','tcp','-noreset'])
        end=time.monotonic()+10
        while not Path(f'/tmp/.X11-unix/X{display_num}').exists():
            if time.monotonic()>end:raise TimeoutError('Xvfb startup')
            run.pause(.1)
        monitor=run.start('monitor',[sys.executable,str(HERE/'dynamic_monitor.py'),'--out',str(out/'run'),'--timeout',str(a.timeout_wall)])
        run.pause(1.)
        run.start('infrastructure',['ros2','launch',str(launch)])
        run.pause(a.startup_seconds)
        docks=PROJECT/'ros2_validation/close_private_docks.py'
        subprocess.run([sys.executable,str(docks),a.display],env=env,check=True,timeout=20)
        run.pause(1.)
        subprocess.run(['/usr/bin/ffmpeg','-hide_banner','-loglevel','error','-y','-f','x11grab','-video_size','1920x1080','-i',a.display+'.0','-frames:v','1',str(out/'preflight.png')],env=env,check=True,timeout=20)
        print(f'PREFLIGHT {out/"preflight.png"}',flush=True)
        if a.record_video:
            run.start('capture',['/usr/bin/ffmpeg','-hide_banner','-nostats','-f','x11grab','-framerate','15','-video_size','1920x1080','-i',a.display+'.0','-an','-c:v','libx264','-preset','veryfast','-crf','23','-pix_fmt','yuv420p','-movflags','+faststart',str(out/'evidencia_dynamic_ros2_rviz.mp4')])
            run.pause(.5)
        run.start('controllers',[sys.executable,str(HERE/'dynamic_controller.py'),'--config',str(out/'dynamic_controllers.yaml'),'--out',str(out/'run'),'--ros-args','-p','use_sim_time:=true'])
        manifest['status']='running';manifest['commands']=run.records;save();progress_at=0.
        while monitor.poll() is None:
            if time.monotonic()-started>a.timeout_wall:raise TimeoutError('Dynamic run wall timeout')
            for name,proc in run.processes:
                if name!='monitor' and proc.poll() is not None:raise RuntimeError(f'{name} exited {proc.returncode}')
            if time.monotonic()>progress_at:
                counts={}
                for name in ('ego','opp'):
                    path=out/'run'/f'laps_{name}.csv';counts[name]=max(0,len(path.read_text().splitlines())-1) if path.exists() else 0
                print(f'PROGRESS wall={time.monotonic()-started:.1f}s laps={counts}',flush=True);progress_at=time.monotonic()+30
            time.sleep(.25)
        summary=json.loads((out/'run/summary_dynamic.json').read_text());manifest['summary']=summary
        completed=summary['completed'] is True;manifest['status']='completed' if completed else 'incomplete'
    except KeyboardInterrupt:error='interrupted';manifest.update(status='interrupted',error=error)
    except Exception as exc:error=f'{type(exc).__name__}: {exc}';manifest.update(status='failed',error=error);print(error,file=sys.stderr,flush=True)
    finally:
        manifest['commands']=run.records;manifest['process_exit']=run.stop();manifest['wall_seconds']=time.monotonic()-started
        manifest['ended_utc']=datetime.now(timezone.utc).isoformat();save()
    if completed:
        command=[sys.executable,str(HERE/'validate_dynamic.py'),'--run',str(out/'run'),'--config',str(out/'dynamic_controllers.yaml'),'--map-yaml',str(map_yaml),'--output',str(out/'checks_dynamic.json')]
        audit=subprocess.run(command,env=env,stdout=(out/'logs/audit.log').open('w'),stderr=subprocess.STDOUT)
        manifest['audit_exit']=audit.returncode;save()
        return audit.returncode
    return 1
if __name__=='__main__':raise SystemExit(main())
