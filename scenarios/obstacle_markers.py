"""Identifica en RViz los obstáculos que YA están en el mapa de ocupación."""
import sys
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
import yaml


class ObstacleLabels(Node):
    def __init__(self, configuration):
        super().__init__('static_obstacle_labels')
        self.config = yaml.safe_load(Path(configuration).read_text())
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(MarkerArray, '/obstacles/markers', qos)
        self.create_timer(1., self.publish)
        self.publish()

    def publish(self):
        array = MarkerArray()
        for i, obstacle in enumerate(self.config['obstacles']):
            body = Marker()
            body.header.frame_id = 'map'
            body.ns, body.id, body.type = 'obstacle_footprints', i, Marker.CYLINDER
            body.pose.orientation.w = 1.
            body.pose.position.x, body.pose.position.y = float(obstacle['x']), float(obstacle['y'])
            body.pose.position.z = .2
            body.scale.x = body.scale.y = 2 * float(obstacle['radius'])
            body.scale.z = .4
            body.color.r, body.color.a = 1., 1.
            array.markers.append(body)
            label = Marker()
            label.header.frame_id = 'map'
            label.ns, label.id, label.type = 'obstacle_names', i, Marker.TEXT_VIEW_FACING
            label.pose.orientation.w = 1.
            label.pose.position.x, label.pose.position.y = float(obstacle['x']) + 2., float(obstacle['y'])
            label.pose.position.z, label.scale.z = 1., 1.6
            label.color.r, label.color.a = 1., 1.
            label.text = obstacle['id']
            array.markers.append(label)
        self.publisher.publish(array)


def main():
    rclpy.init()
    node = ObstacleLabels(sys.argv[1])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
