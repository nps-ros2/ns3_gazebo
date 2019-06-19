#!/usr/bin/env python3
from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.set_nns import set_nns

class GS(Node):

    def __init__(self):
        super().__init__('groundstation')
        self.sub = self.create_subscription(String, 'odometry',
                                            self.odometry_callback)
        self.sub = self.create_subscription(String, 'image',
                                            self.image_callback)

    def odometry_callback(self, msg):
        self.get_logger().info('Odometry: [%s]' % msg.data)
    def image_callback(self, msg):
        self.get_logger().info('Image size: [%d]' % len(msg.data))

def main(args=None):
    parser = ArgumentParser(description="Testbed ground station (GS).")
    parser.add_argument("-n", "--use_nns", action="store_true",
                        help="Run in its own Network Namespace")
    args = parser.parse_args()

    if args.use_nns:
        set_nns("nns1")

    rclpy.init()

    node = GS()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

