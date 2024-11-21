#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LaserScanFrameFixer(Node):
    def __init__(self):
        super().__init__('laser_scan_frame_fixer')
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, '/scan_fixed', 10)

    def callback(self, msg):
        msg.header.frame_id = 'laser_scanner_ray_link'  # Correct frame ID
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanFrameFixer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
