#!/usr/bin/env python3
from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.testbed_codec import testbed_decode

class GS(Node):

    def __init__(self):
        super().__init__('groundstation')
        self.sub = self.create_subscription(String, 'odometry',
                                            self.subscription_callback)
        self.sub = self.create_subscription(String, 'image',
                                            self.subscription_callback)

    def subscription_callback(self, msg):
        source, name, number, size, dt = testbed_decode(msg.data)
        response = "%s,%s,%d,%d,%f"%(source, name, number, size, dt)
        self.get_logger().info(response)

def main(args=None):
    parser = ArgumentParser(description="Testbed ground station (GS).")
    args = parser.parse_args()

    rclpy.init()

    node = GS()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

