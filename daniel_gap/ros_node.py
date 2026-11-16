"""Adaptador ROS 2: scan/odom -> drive; estado y CSV de vueltas."""
import csv, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import String, Bool, Float64
from .core import GapController, LapCounter
from .fast_core import FastGapController
import math

class GapNode(Node):
    def __init__(self):
        super().__init__('daniel_gap')
        defaults={'max_speed':2.5,'controller_profile':'conservative',
                  'gain':.7,'lateral_accel':12.,'brake_accel':8.,'smooth':31,'alpha':.6,
                  'start_x':0.5,'start_y':15.,'start_yaw':-math.pi/2,
                  'target_laps':10,'csv_path':'laps.csv','scan_topic':'/scan',
                  'odom_topic':'/ego_racecar/odom','drive_topic':'/drive'}
        for k,v in defaults.items(): self.declare_parameter(k,v)
        p=lambda k:self.get_parameter(k).value
        if not 0. < p('max_speed') <= 20.:
            raise ValueError('max_speed debe estar entre 0 y 20 m/s (límite físico)')
        self.fast_profile=p('controller_profile') == 'fast'
        if self.fast_profile:
            self.control=FastGapController(p('max_speed'), gain=p('gain'),
                lateral_accel=p('lateral_accel'), brake_accel=p('brake_accel'),
                smooth=p('smooth'), alpha=p('alpha'))
        elif p('controller_profile') == 'conservative':
            self.control=GapController(p('max_speed'))
        else:
            raise ValueError('controller_profile debe ser conservative o fast')
        self.counter=LapCounter(p('start_x'),p('start_y'),p('start_yaw'))
        self.target=p('target_laps'); self.last_scan=None; self.last_scan_stamp=None; self.halted=False
        self.finished=False
        self.last_command=(0.,0.)
        self.pub=self.create_publisher(AckermannDriveStamped,p('drive_topic'),10)
        self.status=self.create_publisher(String,'/daniel_gap/status',10)
        self.origin_pub=self.create_publisher(Float64,'/daniel_gap/timing_origin',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan,p('scan_topic'),self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,p('odom_topic'),self.odom,qos_profile_sensor_data)
        # El bridge del curso NO publica colisión. Requiere detector externo para certificarla.
        self.create_subscription(Bool,'/daniel_gap/collision',self.collision,10)
        self.create_timer(.1,self.watchdog)
        path=Path(p('csv_path')).expanduser(); path.parent.mkdir(parents=True,exist_ok=True)
        self.file=path.open('w',newline=''); self.writer=csv.writer(self.file)
        self.writer.writerow(['lap','seconds']); self.file.flush()

    def send(self,steer=0.,speed=0.):
        msg=AckermannDriveStamped(); msg.header.stamp=self.get_clock().now().to_msg()
        msg.drive.steering_angle=float(steer); msg.drive.speed=float(speed); self.pub.publish(msg)

    def scan(self,msg):
        self.last_scan=time.monotonic()
        stamp=(msg.header.stamp.sec,msg.header.stamp.nanosec)
        # El bridge puede repetir el mismo escaneo entre pasos físicos.
        # El filtro rápido debe avanzar una sola vez por medición nueva.
        if self.fast_profile and stamp == self.last_scan_stamp:
            # Reenviar permite recuperar una orden perdida durante discovery.
            self.send(*((self.last_command[0],0.) if self.halted else self.last_command))
            return
        self.last_scan_stamp=stamp
        if not self.halted or (self.finished and not self.counter.invalid):
            steer,speed=self.control.command(msg.ranges,msg.angle_min,msg.angle_increment)
            # Al cruzar la décima meta, frenar conservando seguimiento lateral.
            # Enderezar de golpe todavía en movimiento puede causar contacto.
            self.last_command=(steer,0. if self.halted else speed)
        else:
            self.last_command=(self.last_command[0],0.)
        self.send(*self.last_command)

    def collision(self,msg):
        if msg.data:
            self.counter.invalid=True; self.halted=True; self.send(self.last_command[0],0.)

    def watchdog(self):
        if self.last_scan is None or time.monotonic()-self.last_scan>.5: self.send()

    def odom(self,msg):
        p=msg.pose.pose.position; stamp=msg.header.stamp
        t=stamp.sec+stamp.nanosec*1e-9
        first=self.counter.previous is None
        duration=self.counter.update(p.x,p.y,t)
        if first:
            self.origin_pub.publish(Float64(data=t))
        if duration is not None:
            self.writer.writerow([len(self.counter.times),duration]); self.file.flush()
            self.get_logger().info(f'Vuelta {len(self.counter.times)}: {duration:.3f} s')
        if len(self.counter.times)>=self.target or self.counter.invalid:
            self.finished=len(self.counter.times)>=self.target and not self.counter.invalid
            self.halted=True; self.send(self.last_command[0],0.)
        elapsed=0. if self.counter.started is None else t-self.counter.started
        text=f'Daniel Bejarano | Sakhir | Vueltas {len(self.counter.times)}/{self.target} | Actual {elapsed:.2f}s | Tiempos {self.counter.times} | Invalido {self.counter.invalid}'
        self.status.publish(String(data=text))

def main():
    rclpy.init(); node=GapNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if rclpy.ok(): node.send()
        node.file.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
