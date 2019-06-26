#!/usr/bin/env python3
from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.testbed_codec import testbed_decode
from ns3_testbed_nodes.pipe_logger import PipeLogger, PIPE_NAME

class GS(Node):

    def __init__(self, use_pipe_logger):
        super().__init__('groundstation')
        self.use_pipe_logger = use_pipe_logger
        if use_pipe_logger:
            self.pipe_logger = PipeLogger()

        self.sub = self.create_subscription(String, 'odometry',
                                            self.subscription_callback)
        self.sub = self.create_subscription(String, 'image',
                                            self.subscription_callback)

    def subscription_callback(self, msg):
        source, name, number, size, dt = testbed_decode(msg.data)
        response = "%s,%s,%d,%d,%f"%(source, name, number, size, dt)
        if self.use_pipe_logger:
            self.pipe_logger.log(response)
        else:
            # use Node's native logger
            self.get_logger().info(response)

def main():
    parser = ArgumentParser(description="Testbed ground station (GS).")
    parser.add_argument("-p","--use_pipe_logger", action="store_true",
                        help="Send output to pipe '%s'"%PIPE_NAME)
    args = parser.parse_args()

    rclpy.init()

    node = GS(args.use_pipe_logger)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

