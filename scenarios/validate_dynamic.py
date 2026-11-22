"""Replay the received two-car ROS bag; audit laps, collisions and a real pass.

This reuses the same finish-gate algorithm on a separate odometry replay. It
is a recount, not an independent physical clock. Geometric map checks evaluate
vehicle centres; simulator collision flags cover the simulated bodies.
"""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
import yaml
import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Float64, String
from rosgraph_msgs.msg import Clock
PROJECT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PROJECT));sys.path.insert(0,str(PROJECT/'ros2_validation'))
from daniel_gap.core import LapCounter
from validate_run import map_regions,grid_coordinates
from dynamic_controller import TOPICS
from dynamic_metrics import OvertakeDetector
from validate_static import coverage

def audit(a):
    out=a.run.resolve();cfg=yaml.safe_load(a.config.read_text())
    summary=json.loads((out/'summary_dynamic.json').read_text())
    manifest=json.loads((out.parent/'manifest.json').read_text())
    gate=cfg['gate'];counts={};cars={};clock=None;clock_backwards=0;finite=True;pending={};pairs=[]
    for name in TOPICS:
        rows=list(csv.DictReader((out/f'laps_{name}.csv').open()))
        cars[name]={'csv_times':[float(r['seconds']) for r in rows],'csv_indices':[int(r['lap']) for r in rows],
            'origins':[],'odom':[],'positions':[],'positive_flags':0,'max_command_speed':0.,'max_command_steer':0.,
            'last_command_speed':None,'final_measured_vx':None,'max_measured_vx':0.,'first_pose':None}
    reader=rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
    while reader.has_next():
        topic,data,_=reader.read_next();counts[topic]=counts.get(topic,0)+1
        if topic=='/clock':
            m=deserialize_message(data,Clock);stamp=m.clock.sec*10**9+m.clock.nanosec
            if clock is not None and stamp<clock:clock_backwards+=1
            clock=stamp
        for name,t in TOPICS.items():
            car=cars[name]
            if topic==t['collision']:car['positive_flags']+=int(deserialize_message(data,Bool).data)
            elif topic==f'/daniel_dynamic/{name}/timing_origin':car['origins'].append(float(deserialize_message(data,Float64).data))
            elif topic==t['drive']:
                m=deserialize_message(data,AckermannDriveStamped)
                speed=float(m.drive.speed);steer=float(m.drive.steering_angle)
                finite &= math.isfinite(speed) and math.isfinite(steer)
                car['max_command_speed']=max(car['max_command_speed'],abs(speed));car['max_command_steer']=max(car['max_command_steer'],abs(steer));car['last_command_speed']=speed
            elif topic==t['odom']:
                m=deserialize_message(data,Odometry);p=m.pose.pose.position;q=m.pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9;vx=float(m.twist.twist.linear.x)
                finite &= all(math.isfinite(v) for v in (p.x,p.y,yaw,stamp,vx))
                sample=(stamp,float(p.x),float(p.y),yaw,vx)
                car['odom'].append(sample);car['positions'].append((p.x,p.y));car['final_measured_vx']=vx
                car['max_measured_vx']=max(car['max_measured_vx'],abs(vx))
                if car['first_pose'] is None:car['first_pose']=sample
                key=round(stamp*1e9);pending.setdefault(key,{})[name]=sample
                if len(pending[key])==2:
                    pair=pending.pop(key);pairs.append((stamp,pair['ego'],pair['opp']))
    regions,map_info=map_regions(a.map_yaml)
    rr,cc=grid_coordinates(gate['x'],gate['y'],map_info)
    track=regions[int(rr),int(cc)]
    if track==0:raise ValueError('Common start gate is not free track')
    checks={'native_ros_runtime':manifest['native_modules']['rclpy']['path'].startswith('/opt/ros/humble/'),
            'no_physics_overrides':manifest.get('physics_overrides')=={},
            'clock_monotonic':counts.get('/clock',0)>100 and clock_backwards==0,
            'recorded_numbers_finite':bool(finite),
            'manifest_completed_and_closed':manifest.get('status')=='completed' and bool(manifest.get('ended_utc')),
            'summary_completed':summary.get('completed') is True}
    resultcars={}
    reference_config=yaml.safe_load((Path(__file__).with_name('static_obstacles.yaml')).read_text())
    references=reference_config['obstacles']
    for name,t in TOPICS.items():
        car=cars[name];counter=LapCounter(gate['x'],gate['y'],gate['yaw']);origins=car['origins']
        origin=origins[0] if origins else None
        if origin is None:raise ValueError(f'Missing timing origin: {name}')
        consistent=all(math.isfinite(v) and abs(v-origin)<1e-8 for v in origins)
        for stamp,x,y,_,_ in car['odom']:
            if stamp>=origin-1e-8:counter.update(x,y,stamp,car['positive_flags']>0)
        csvtimes=car['csv_times'];status=summary['vehicles'][name];reported=status['laps']
        points=np.asarray(car['positions']);rows,cols=grid_coordinates(points[:,0],points[:,1],map_info)
        h,w=regions.shape;inside=(np.isfinite(rows)&np.isfinite(cols)&(rows>=0)&(cols>=0)&(rows<h)&(cols<w))
        on=np.zeros(len(points),dtype=bool);on[inside]=regions[rows[inside].astype(int),cols[inside].astype(int)]==track
        agree=(len(csvtimes)==len(counter.times) and all(abs(x-y)<=a.lap_tolerance for x,y in zip(csvtimes,counter.times)))
        reportagree=(len(csvtimes)==len(reported) and all(abs(x-y)<1e-7 for x,y in zip(csvtimes,reported)))
        visits=coverage([(stamp,x,y) for stamp,x,y,_,_ in car['odom']],origins,csvtimes,references,distance=3.)
        # These coordinates identify track sectors only: the dynamic map has no added obstacles.
        for visit in visits:
            visit['passed_all_five_sectors_in_order']=visit.pop('passed_all_five_in_order')
            for event in visit['events']:event['sector_reference']=event.pop('obstacle')
        checks.update({
            f'{name}_each_actual_lap_visits_five_track_sectors_in_order':len(visits)==len(csvtimes) and len(visits)>=10 and all(v['passed_all_five_sectors_in_order'] for v in visits),
            f'{name}_at_least_ten_real_laps':len(csvtimes)>=10,
            f'{name}_all_actual_laps_csv_and_summary_agree':reportagree,
            f'{name}_all_actual_laps_odom_recount_agrees':agree,
            f'{name}_recount_not_invalid':not counter.invalid,
            f'{name}_lap_indices_sequential':car['csv_indices']==list(range(1,len(csvtimes)+1)),
            f'{name}_lap_times_positive':bool(csvtimes) and all(math.isfinite(x) and x>0 for x in csvtimes),
            f'{name}_timing_origin_present_consistent':bool(origins) and consistent,
            f'{name}_collision_messages_present':counts.get(t['collision'],0)>=100,
            f'{name}_no_collision_flags':car['positive_flags']==0,
            f'{name}_scan_evidence_present':counts.get(t['scan'],0)>=10,
            f'{name}_command_speed_within_limit':counts.get(t['drive'],0)>0 and car['max_command_speed']<=float(np.float32(cfg[name]['max_speed']))+1e-6,
            f'{name}_command_steer_within_limit':counts.get(t['drive'],0)>0 and car['max_command_steer']<=float(np.float32(.4189))+1e-6,
            f'{name}_all_odom_centres_on_original_track':bool(len(points)) and bool(on.all()),
            f'{name}_last_command_zero':car['last_command_speed']==0.,
            f'{name}_physically_stopped':car['final_measured_vx'] is not None and abs(car['final_measured_vx'])<.05,
            f'{name}_first_ten_not_extras_used_for_target':status.get('first_ten_laps')==csvtimes[:10],
        })
        resultcars[name]={k:v for k,v in car.items() if k not in ('odom','positions')}
        resultcars[name].update({'actual_lap_count':len(csvtimes),'target_lap_count':10,'extra_lap_count':max(0,len(csvtimes)-10),
            'first_ten_laps':csvtimes[:10],'extra_laps':csvtimes[10:],'best_first_ten':min(csvtimes[:10]) if csvtimes else None,
            'recount_times':counter.times,'recount_deltas':[y-x for x,y in zip(csvtimes,counter.times)],
            'odom_positions_checked':len(points),'outside_track':int((~on).sum()),'full_route_sector_coverage':visits})
    initial=[cars[n]['first_pose'] for n in ('ego','opp')]
    gap=math.hypot(initial[0][1]-initial[1][1],initial[0][2]-initial[1][2])
    checks['initial_centre_gap_five_metres']=abs(gap-5.)<1e-5
    checks['initial_poses_match_config']=all(all(abs(initial[i][j+1]-cfg[n]['start'][j])<1e-5 for j in range(3)) for i,n in enumerate(('ego','opp')))
    # Recount passes from synchronised paired odometry, not controller status.
    detector=OvertakeDetector();last=None;unique=[]
    for stamp,e,o in sorted(pairs,key=lambda p:p[0]):
        if last is not None and stamp<=last:continue
        last=stamp;unique.append((stamp,e,o));detector.update(stamp,e[1:4],o[1:4])
    checks['synchronised_paired_odometry_present']=len(unique)>100
    checks['at_least_one_measured_ego_overtake']=len(detector.events)>0
    checks['observer_reports_measured_overtake']=bool(summary.get('overtakes'))
    stopped_start=None;last_pair=unique[-1][0] if unique else None
    for stamp,e,o in unique:
        if abs(e[4])<.05 and abs(o[4])<.05:
            if stopped_start is None:stopped_start=stamp
        else:stopped_start=None
    stopped_seconds=last_pair-stopped_start if last_pair is not None and stopped_start is not None else None
    checks['both_physically_stopped_four_sim_seconds']=stopped_seconds is not None and stopped_seconds>=3.95
    image=Path(map_info['image'])
    checks['dynamic_map_matches_official_original_image']=hashlib.sha256(image.read_bytes()).hexdigest()==reference_config['source_image_sha256']
    checks['map_image_hash_unchanged_since_run']=hashlib.sha256(image.read_bytes()).hexdigest()==manifest['map_image_sha256']
    checks['map_yaml_hash_unchanged_since_run']=hashlib.sha256(a.map_yaml.read_bytes()).hexdigest()==manifest['map_yaml_sha256']
    return {'passed':all(checks.values()),'checks':checks,'check_count':len(checks),'run':str(out),'vehicles':resultcars,
        'bag_messages':counts,'initial_centre_gap':gap,'paired_samples':len(unique),'overtakes_recounted':detector.events,
        'both_stopped_sim_seconds_recounted':stopped_seconds,'minimum_centre_separation':min(math.hypot(e[1]-o[1],e[2]-o[2]) for _,e,o in unique) if unique else None,
        'finish_protocol':summary['finish_protocol'],'map':map_info,'lap_tolerance_seconds':a.lap_tolerance,
        'sector_reference_note':'Five coordinates from the static scenario are measurement-only track-sector references; no obstacles are inserted into the original dynamic map. Each complete actual lap must pass them in order within 3 metres.',
        'auditor_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'scope':'Recount uses the same LapCounter on separately read received DDS odometry; simulation clock, not independent physical time. Original-map geometry checks vehicle centres; no-collision claim uses both complete received flag streams. Every actual extra lap remains explicit.'}

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--run',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--map-yaml',type=Path,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--lap-tolerance',type=float,default=.1)
    a=p.parse_args()
    if a.output.exists():
        raise FileExistsError('No se sobrescribe una auditoría previa; elige otra ruta --output.')
    try:result=audit(a)
    except Exception as e:result={'passed':False,'error':f'{type(e).__name__}: {e}'}
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
