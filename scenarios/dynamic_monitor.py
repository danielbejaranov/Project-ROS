"""DDS observer, rosbag writer and RViz text for two real simulator vehicles."""
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.serialization import serialize_message
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String, Float64
from rosgraph_msgs.msg import Clock
from visualization_msgs.msg import Marker
import rosbag2_py
from dynamic_controller import TOPICS
from dynamic_metrics import OvertakeDetector

class Monitor(Node):
    def __init__(self,out,timeout):
        super().__init__('daniel_dynamic_monitor')
        self.out=out; out.mkdir(parents=True,exist_ok=True)
        self.started_wall=time.monotonic(); self.timeout=timeout; self.sim=0.
        self.finished=False; self.reason=None; self.stopped_since=None
        self.counts={}; self.states={n:None for n in TOPICS}; self.origins={n:None for n in TOPICS}
        self.collisions={n:False for n in TOPICS}; self.poses={n:None for n in TOPICS}
        self.initial_poses={n:None for n in TOPICS}; self.commands={n:(0.,0.) for n in TOPICS}
        self.actual_speed={n:None for n in TOPICS}; self.paths={n:PathMsg() for n in TOPICS}
        self.detector=OvertakeDetector()
        self.writer=rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        self.files={}; self.csv={}; self.pathpub={}
        self.overlay=self.create_publisher(Marker,'/daniel_dynamic/overlay',10)
        specs=[('/clock',Clock,'rosgraph_msgs/msg/Clock',10)]
        for name,topics in TOPICS.items():
            self.files[name]=(out/f'telemetry_{name}.csv').open('w',newline='')
            self.csv[name]=csv.writer(self.files[name]); self.csv[name].writerow(['sim_time','x','y','yaw','measured_vx','command_speed','laps'])
            self.pathpub[name]=self.create_publisher(PathMsg,f'/daniel_dynamic/{name}/path',5)
            specs.extend([(topics['scan'],LaserScan,'sensor_msgs/msg/LaserScan',qos_profile_sensor_data),
                (topics['odom'],Odometry,'nav_msgs/msg/Odometry',qos_profile_sensor_data),
                (topics['drive'],AckermannDriveStamped,'ackermann_msgs/msg/AckermannDriveStamped',10),
                (topics['collision'],Bool,'std_msgs/msg/Bool',10),
                (f'/daniel_dynamic/{name}/status',String,'std_msgs/msg/String',10),
                (f'/daniel_dynamic/{name}/timing_origin',Float64,'std_msgs/msg/Float64',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))])
        for topic,cls,typename,qos in specs:
            self.counts[topic]=0
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=typename,serialization_format='cdr'))
            self.create_subscription(cls,topic,lambda msg,t=topic:self.receive(t,msg),qos)
        self.create_timer(.1,self.display)

    def write_message(self,topic,msg):
        encoded=serialize_message(msg)
        for attempt in range(20):
            try:
                self.writer.write(topic,encoded,max(0,round(self.sim*1e9)))
                return
            except RuntimeError as error:
                if 'database is locked' not in str(error) or attempt==19:
                    raise
                time.sleep(.01)

    def receive(self,topic,msg):
        self.counts[topic]+=1
        is_scan=topic in [t['scan'] for t in TOPICS.values()]
        if not is_scan or self.counts[topic]%100==1:
            self.write_message(topic,msg)
        if topic=='/clock':self.sim=msg.clock.sec+msg.clock.nanosec*1e-9
        for name,topics in TOPICS.items():
            if topic==topics['drive']:self.commands[name]=(msg.drive.steering_angle,msg.drive.speed)
            elif topic==topics['collision']:self.collisions[name]|=bool(msg.data)
            elif topic==f'/daniel_dynamic/{name}/timing_origin':self.origins[name]=msg.data
            elif topic==f'/daniel_dynamic/{name}/status':
                old=self.states[name]
                self.states[name]=json.loads(msg.data)
                if old is None or len(old['laps'])!=len(self.states[name]['laps']):
                    print(f"{name.upper()} laps={len(self.states[name]['laps'])}/10 collision={self.collisions[name]}",flush=True)
            elif topic==topics['odom']:
                p=msg.pose.pose.position; q=msg.pose.pose.orientation
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.poses[name]=(stamp,p.x,p.y,yaw)
                if self.initial_poses[name] is None:self.initial_poses[name]=list(self.poses[name])
                self.actual_speed[name]=float(msg.twist.twist.linear.x)
                if self.counts[topic]%10==1:
                    laps=len(self.states[name]['laps']) if self.states[name] else 0
                    self.csv[name].writerow([stamp,p.x,p.y,yaw,self.actual_speed[name],self.commands[name][1],laps])
                if self.counts[topic]%50==1:
                    pose=PoseStamped(); pose.header=msg.header; pose.pose=msg.pose.pose
                    self.paths[name].header=msg.header;self.paths[name].poses.append(pose)
                if all(self.poses.values()) and abs(self.poses['ego'][0]-self.poses['opp'][0])<1e-8:
                    old=len(self.detector.events)
                    self.detector.update(stamp,self.poses['ego'][1:],self.poses['opp'][1:])
                    if len(self.detector.events)>old:
                        print('OVERTAKE '+json.dumps(self.detector.events[-1]),flush=True)
        if any(self.collisions.values()):self.reason='collision';self.finished=True
        if any(s and s.get('invalid') for s in self.states.values()):self.reason='invalid_counter';self.finished=True
        both_done=all(s and s.get('finished') and len(s.get('laps',[]))>=10 for s in self.states.values())
        both_stopped=all(v is not None and abs(v)<.05 for v in self.actual_speed.values())
        if both_done and both_stopped:
            if self.stopped_since is None:self.stopped_since=self.sim
            if self.sim-self.stopped_since>=4.:
                self.reason='completed';self.finished=True
        else:self.stopped_since=None

    def display(self):
        for name in TOPICS:self.pathpub[name].publish(self.paths[name])
        lines=['Daniel Bejarano | Sakhir original | ROS 2: dos vehiculos']
        for name in TOPICS:
            s=self.states[name];laps=s['laps'] if s else []
            elapsed=s.get('current_lap_seconds',0.) if s else 0.
            text=f'{name.upper()}: {len(laps)}/10 minimo | Actual: {elapsed:.2f} s'
            if laps:text+=f' | Mejor: {min(laps):.2f} s'
            text+=f' | Colision: {"SI" if self.collisions[name] else "NO"}'
            lines.append(text)
        lines.append(f'Adelantamientos medidos de EGO: {len(self.detector.events)}')
        m=Marker();m.header.frame_id='map';m.ns='dynamic';m.id=0
        m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD;m.pose.orientation.w=1.
        m.pose.position.x=30.;m.pose.position.y=70.;m.pose.position.z=.2
        m.scale.z=2.6;m.color.r=.2;m.color.g=.9;m.color.b=1.;m.color.a=1.;m.text='\n'.join(lines)
        self.overlay.publish(m)
        progress={'sim_seconds':self.sim,'laps':{name:len(self.states[name]['laps']) if self.states[name] else 0 for name in TOPICS},'collisions':self.collisions,'overtakes':len(self.detector.events),'wall_seconds':time.monotonic()-self.started_wall}
        temporary=self.out/'progress_live.tmp'
        temporary.write_text(json.dumps(progress)+'\n');temporary.replace(self.out/'progress_live.json')
        if time.monotonic()-self.started_wall>self.timeout:
            self.reason='wall_timeout';self.finished=True

    def close(self):
        for f in self.files.values():f.close()
        initial=self.initial_poses
        gap=math.hypot(initial['ego'][1]-initial['opp'][1],initial['ego'][2]-initial['opp'][2]) if all(initial.values()) else None
        completed=self.reason=='completed' and bool(self.detector.events) and not any(self.collisions.values())
        result={'runtime':'native ROS 2 Humble / DDS','simulator':'unchanged F1TENTH Gym dynamics, two agents',
            'completed':completed,'reason':self.reason,'finish_protocol':'both continue unchanged FTG until each has at least ten full laps; all extra laps retained, then simultaneous zero-speed commands and four measured stopped simulation seconds','vehicles':self.states,'collisions':self.collisions,
            'origins':self.origins,'received_messages':self.counts,'initial_poses':initial,
            'initial_centre_gap':gap,'overtakes':self.detector.events,'sim_seconds':self.sim,
            'stopped_since':self.stopped_since,'stopped_sim_seconds':None if self.stopped_since is None else self.sim-self.stopped_since,
            'final_measured_vx':self.actual_speed,'wall_seconds':time.monotonic()-self.started_wall,
            'bag_scan_sampling':'1/100 per vehicle; all other received messages listed are written',
            'coverage_note':'received DDS messages, not an assertion that every simulator publication reached this observer',
            'nodes':self.get_node_names_and_namespaces() if rclpy.ok() else [],
            'topics':self.get_topic_names_and_types() if rclpy.ok() else []}
        (self.out/'summary_dynamic.json').write_text(json.dumps(result,indent=2)+'\n')
        (self.out/'overtakes.json').write_text(json.dumps(self.detector.events,indent=2)+'\n')
        del self.writer
        print(json.dumps(result,indent=2),flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--out',type=Path,required=True);p.add_argument('--timeout',type=float,default=1800.)
    a,rosargs=p.parse_known_args();rclpy.init(args=rosargs);node=Monitor(a.out,a.timeout)
    try:
        while rclpy.ok() and not node.finished:rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt:node.reason='interrupted'
    except Exception as error:
        node.reason=f'error: {type(error).__name__}: {error}'
        raise
    finally:
        node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
