"""Estado visible y trayectoria de la demostración interactiva ROS 2."""
import ast,re
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
class LiveStatus(Node):
    def __init__(self):
        super().__init__('daniel_live_status');self.text='Daniel Bejarano | ROS 2\nEsperando controlador'
        self.path=Path();self.count=0
        self.pub=self.create_publisher(Marker,'/daniel_gap/overlay',5)
        self.path_pub=self.create_publisher(Path,'/daniel_gap/path',5)
        self.create_subscription(String,'/daniel_gap/status',self.status,10)
        self.create_subscription(Odometry,'/ego_racecar/odom',self.odom,qos_profile_sensor_data)
        self.create_timer(.1,self.display)
    def status(self,msg):
        fields=msg.data.split('|');self.text='Daniel Bejarano | Sakhir | ROS 2\n'+fields[2].strip()+' | '+fields[3].strip()+'\n'+fields[-1].strip()
    def odom(self,msg):
        self.count+=1
        if self.count%50==1:
            p=PoseStamped();p.header=msg.header;p.pose=msg.pose.pose;self.path.header=msg.header;self.path.poses.append(p)
    def display(self):
        m=Marker();m.header.frame_id='map';m.ns='validation';m.id=0;m.type=Marker.TEXT_VIEW_FACING;m.action=Marker.ADD
        m.pose.position.x=30.;m.pose.position.y=70.5;m.pose.position.z=.2;m.pose.orientation.w=1.
        m.scale.z=3.;m.color.r=.1;m.color.g=.8;m.color.b=1.;m.color.a=1.;m.text=self.text
        self.pub.publish(m);self.path_pub.publish(self.path)
def main():
    rclpy.init();n=LiveStatus()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
