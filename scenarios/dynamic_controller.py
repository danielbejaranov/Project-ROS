"""Two independent Part 1 FTG policies over native ROS 2 topics.

Only each vehicle's own LaserScan reaches its policy. Odometry supplies the
common finish-gate measurement and guided stop, never a steering/speed policy.
"""
import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String, Float64

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from daniel_gap.core import LapCounter
from daniel_gap.fast_core import FastGapController

TOPICS = {
    'ego': {'scan': '/scan', 'odom': '/ego_racecar/odom', 'drive': '/drive', 'collision': '/daniel_gap/collision'},
    'opp': {'scan': '/opp_scan', 'odom': '/opp_racecar/odom', 'drive': '/opp_drive', 'collision': '/daniel_gap/opp_collision'},
}

class DualController(Node):
    def __init__(self, config, out):
        super().__init__('daniel_dynamic_controllers')
        self.config = config
        self.target = int(config['target_laps'])
        self.cars = {}
        self.failed = False
        self.stop_all = False
        out.mkdir(parents=True, exist_ok=True)
        gate = config['gate']
        for name in ('ego', 'opp'):
            p = config[name]
            speed = float(p['max_speed'])
            if not 0 < speed <= 20.:
                raise ValueError('Speed limit outside physical 0..20 m/s')
            car = {'counter': LapCounter(gate['x'], gate['y'], gate['yaw']),
                   'control': FastGapController(**{k:p[k] for k in ('max_speed','gain','lateral_accel','brake_accel','smooth','alpha')}),
                   'last_scan_wall': None, 'last_scan_stamp': None, 'last_command': (0.,0.),
                   'finished': False, 'collision': False, 'origin': None}
            car['drive_pub'] = self.create_publisher(AckermannDriveStamped, TOPICS[name]['drive'], 10)
            car['origin_pub'] = self.create_publisher(Float64, f'/daniel_dynamic/{name}/timing_origin',
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            car['status_pub'] = self.create_publisher(String, f'/daniel_dynamic/{name}/status', 10)
            car['file'] = (out / f'laps_{name}.csv').open('w', newline='')
            car['csv'] = csv.writer(car['file']); car['csv'].writerow(['lap','seconds']); car['file'].flush()
            self.cars[name] = car
            self.create_subscription(LaserScan, TOPICS[name]['scan'], lambda msg,n=name:self.scan(n,msg), qos_profile_sensor_data)
            self.create_subscription(Odometry, TOPICS[name]['odom'], lambda msg,n=name:self.odom(n,msg), qos_profile_sensor_data)
            self.create_subscription(Bool, TOPICS[name]['collision'], lambda msg,n=name:self.collision(n,msg), 10)
        self.create_timer(.1, self.watchdog)

    def send(self, name, steering, speed):
        msg = AckermannDriveStamped(); msg.header.stamp = self.get_clock().now().to_msg()
        msg.drive.steering_angle = float(steering); msg.drive.speed = float(speed)
        self.cars[name]['drive_pub'].publish(msg)

    def scan(self, name, msg):
        car = self.cars[name]
        car['last_scan_wall'] = time.monotonic()
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp != car['last_scan_stamp']:
            car['last_scan_stamp'] = stamp
            # Completed cars retain LiDAR lateral following during physical braking.
            # A collision stops both vehicles immediately.
            if not self.failed:
                steer, speed = car['control'].command(msg.ranges, msg.angle_min, msg.angle_increment)
                car['last_command'] = (steer, 0. if self.stop_all else speed)
            else:
                car['last_command'] = (car['last_command'][0], 0.)
        self.send(name, *car['last_command'])

    def collision(self, name, msg):
        if msg.data:
            self.cars[name]['collision'] = True
            self.cars[name]['counter'].invalid = True
            self.failed = True
            for n, car in self.cars.items():
                car['last_command'] = (car['last_command'][0], 0.)
                self.send(n, *car['last_command'])

    def odom(self, name, msg):
        car = self.cars[name]; counter = car['counter']; p = msg.pose.pose.position
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        if car['origin'] is None:
            car['origin'] = stamp
            car['origin_pub'].publish(Float64(data=stamp))
        duration = counter.update(p.x, p.y, stamp)
        if duration is not None:
            car['csv'].writerow([len(counter.times),duration]); car['file'].flush()
            print(f'{name.upper()} LAP {len(counter.times)} {duration:.3f}s', flush=True)
        car['finished'] = len(counter.times) >= self.target
        self.stop_all = all(c['finished'] for c in self.cars.values())
        if counter.invalid:
            self.failed = True
        if self.stop_all or self.failed:
            car['last_command'] = (car['last_command'][0],0.)
            self.send(name,*car['last_command'])
        status = {'vehicle': name, 'laps':counter.times, 'completed_laps':len(counter.times),
                  'finished':car['finished'], 'global_stop':self.stop_all, 'target_laps':self.target, 'first_ten_laps':counter.times[:self.target], 'invalid':counter.invalid, 'collision':car['collision'],
                  'origin':car['origin'], 'sim_time':stamp,
                  'current_lap_seconds':0. if counter.started is None else stamp-counter.started,
                  'x':p.x,'y':p.y,'measured_vx':msg.twist.twist.linear.x}
        car['status_pub'].publish(String(data=json.dumps(status)))

    def watchdog(self):
        for name,car in self.cars.items():
            if car['last_scan_wall'] is None or time.monotonic()-car['last_scan_wall']>.5:
                self.send(name,0.,0.)

    def close(self):
        for name,car in self.cars.items():
            if rclpy.ok():self.send(name,car['last_command'][0],0.)
            car['file'].close()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,default=Path(__file__).with_name('dynamic_controllers.yaml'))
    parser.add_argument('--out',type=Path,required=True)
    args,ros_args=parser.parse_known_args()
    rclpy.init(args=ros_args)
    node=DualController(yaml.safe_load(args.config.read_text()),args.out)
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.close(); node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=='__main__':main()
