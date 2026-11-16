#!/usr/bin/env python3
"""Mirror the actual monitor Marker into the close-up RViz camera frame.

Presentation only: subscribes to the monitor's already computed text; no timing,
control, odometry or physics computation and no messages on the command topics.
"""
from copy import deepcopy
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
class FollowHUD(Node):
    def __init__(self):
        super().__init__('daniel_recording_follow_hud')
        self.position=None
        self.create_subscription(Odometry,'/ego_racecar/odom',self.odom,qos_profile_sensor_data)
        self.pub=self.create_publisher(Marker,'/daniel_gap/follow_overlay',5)
        self.create_subscription(Marker,'/daniel_gap/overlay',self.mirror,5)
    def odom(self,msg):
        self.position=msg.pose.pose.position
    def mirror(self,msg):
        if self.position is None:return
        output=deepcopy(msg)
        output.header.frame_id='map'
        output.pose.position.x=self.position.x
        output.pose.position.y=self.position.y+5.4
        output.pose.position.z=.2
        output.scale.z=.25
        self.pub.publish(output)
def main():
    rclpy.init();node=FollowHUD()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
if __name__=='__main__':main()
