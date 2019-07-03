#!/usr/bin/env python3
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.testbed_codec import testbed_decode
from ns3_testbed_nodes.pipe_logger import PipeLogger, PIPE_NAME
from ns3_testbed_nodes.setup_reader import read_setup

class GS(Node):

    def __init__(self):
        super().__init__('groundstation')
        self.pipe_logger = PipeLogger()
        self.sub = self.create_subscription(String, 'odometry',
                                            self.subscription_callback)
        self.sub = self.create_subscription(String, 'image',
                                            self.subscription_callback)

    def subscription_callback(self, msg):
        source, name, number, size, dt = testbed_decode(msg.data)
        response = "%s,%s,%d,%d,%f"%(source, name, number, size, dt)
        self.pipe_logger.log(response)
        ## use Node's native logger
        #self.get_logger().info(response)

def main():
    parser = ArgumentParser(description="Testbed ground station (GS).",
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("-n","--robot_name", type=str,
                        help="The name of this robot node.")
    parser.add_argument("-s","--setup_file", type=str,
                        help="The CSV setup file.",
                        default = "../../csv_setup/example1.csv")
    args = parser.parse_args()

    rclpy.init()

    node = GS(publishers, subscribers)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

