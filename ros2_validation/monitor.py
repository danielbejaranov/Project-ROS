"""Observador ROS real: evidencia DDS, bag, tiempos, trayectoria y texto RViz."""
import ast,csv,json,re,time,sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry,Path as PathMsg
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool,String,Float64
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import rosbag2_py

class Monitor(Node):
    def __init__(self,out):
        super().__init__('daniel_validation_monitor')
        self.out=out; out.mkdir(parents=True,exist_ok=True)
        self.writer=rosbag2_py.SequentialWriter()
        self.writer.open(rosbag2_py.StorageOptions(uri=str(out/'rosbag'),storage_id='sqlite3'),rosbag2_py.ConverterOptions('',''))
        self.counts={}; self.collision=False; self.sim=0.; self.start_wall=time.monotonic()
        self.laps=[]; self.finished=False; self.final_at=None; self.odom_count=0
        self.timing_origin=None
        self.latest_status='Esperando controlador'; self.position=None; self.path=PathMsg()
        self.file=(out/'telemetry_ros.csv').open('w',newline=''); self.csv=csv.writer(self.file)
        self.csv.writerow(['sim_time','x','y','yaw','speed','laps'])
        self.path_pub=self.create_publisher(PathMsg,'/daniel_gap/path',5)
        self.text_pub=self.create_publisher(Marker,'/daniel_gap/overlay',5)
        self.command=(0.,0.)
        types=[('/scan',LaserScan,'sensor_msgs/msg/LaserScan'),('/ego_racecar/odom',Odometry,'nav_msgs/msg/Odometry'),('/drive',AckermannDriveStamped,'ackermann_msgs/msg/AckermannDriveStamped'),('/clock',Clock,'rosgraph_msgs/msg/Clock'),('/daniel_gap/collision',Bool,'std_msgs/msg/Bool'),('/daniel_gap/status',String,'std_msgs/msg/String')]
        types.append(('/daniel_gap/timing_origin',Float64,'std_msgs/msg/Float64'))
        for topic,cls,type_name in types:
            self.counts[topic]=0
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic,type=type_name,serialization_format='cdr'))
            qos=qos_profile_sensor_data if topic in ['/scan','/ego_racecar/odom'] else 10
            if topic=='/daniel_gap/timing_origin':
                qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(cls,topic,lambda msg,t=topic:self.receive(t,msg),qos)
        self.create_timer(.1,self.display)

    def receive(self,topic,msg):
        self.counts[topic]+=1
        # Todos los mensajes de estado/control/odom; LiDAR muestreado 1/100.
        if topic!='/scan' or self.counts[topic]%100==1:
            self.writer.write(topic,serialize_message(msg),max(0,round(self.sim*1e9)))
        if topic=='/clock': self.sim=msg.clock.sec+msg.clock.nanosec*1e-9
        elif topic=='/daniel_gap/timing_origin': self.timing_origin=msg.data
        elif topic=='/drive': self.command=(msg.drive.steering_angle,msg.drive.speed)
        elif topic=='/daniel_gap/collision': self.collision |= msg.data
        elif topic=='/daniel_gap/status':
            self.latest_status=msg.data
            match=re.search(r'Tiempos (\[.*?\])',msg.data)
            if match:
                laps=ast.literal_eval(match.group(1))
                if len(laps)>len(self.laps):
                    print(f'ROS LAP {len(laps)}: {laps[-1]:.3f} s | collision={self.collision}',flush=True)
                self.laps=laps
            if len(self.laps)>=10 and self.final_at is None:self.final_at=time.monotonic()
        elif topic=='/ego_racecar/odom':
            self.odom_count+=1; self.position=msg
            if self.odom_count%10==1:
                p=msg.pose.pose.position;q=msg.pose.pose.orientation
                import math
                yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
                stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
                self.csv.writerow([stamp,p.x,p.y,yaw,self.command[1],len(self.laps)])
                if self.odom_count%50==1:
                    ps=PoseStamped();ps.header=msg.header;ps.pose=msg.pose.pose
                    self.path.header=msg.header;self.path.poses.append(ps)
        if self.collision:self.finished=True
        if self.final_at and time.monotonic()-self.final_at>4:self.finished=True

    def display(self):
        self.path_pub.publish(self.path)
        elapsed=self.sim-(self.timing_origin or 0.)-sum(self.laps)
        text=f'Daniel Bejarano | Sakhir | ROS 2 Humble\nVueltas: {len(self.laps)}/10 | Actual: {max(0,elapsed):.2f} s\n'
        text+=f'Mejor: {min(self.laps):.2f} s | ' if self.laps else ''
        text+=f'Colisiones: {"SI" if self.collision else "NO"}'
        m=Marker();m.header.frame_id='map';m.ns='validation';m.id=0;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
        m.pose.position.x=30.;m.pose.position.y=70.5;m.pose.position.z=.2;m.pose.orientation.w=1.
        m.scale.z=3.0;m.color.r=.1;m.color.g=.8;m.color.b=1.;m.color.a=1.;m.text=text
        self.text_pub.publish(m)

    def close(self):
        self.file.close()
        summary={'runtime':'ROS 2 Humble / Fast DDS','simulator':'F1TENTH Gym ROS bridge','laps':self.laps,'collision':self.collision,
                 'completed':len(self.laps)>=10 and not self.collision,'sim_seconds':self.sim,
                 'timing_origin':self.timing_origin,
                 'wall_seconds':time.monotonic()-self.start_wall,'received_messages':self.counts,
                 'bag_scan_sampling':'1/100; other listed topics complete for this observer','latest_status':self.latest_status,
                 'nodes':self.get_node_names_and_namespaces() if rclpy.ok() else [],
                 'topics':self.get_topic_names_and_types() if rclpy.ok() else []}
        (self.out/'summary_ros.json').write_text(json.dumps(summary,indent=2))
        del self.writer
        print(json.dumps(summary,indent=2),flush=True)

def main():
    rclpy.init();node=Monitor(Path(sys.argv[1]).resolve())
    try:
        while rclpy.ok() and not node.finished and time.monotonic()-node.start_wall<3600:
            rclpy.spin_once(node,timeout_sec=.1)
    except KeyboardInterrupt: pass
    finally:
        node.close();node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__':main()
