#!/usr/bin/env python3
from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.set_nns import set_nns

#IMAGE_LENGTH=640*480*1
IMAGE_LENGTH=5

class R(Node):

    def __init__(self, _robot_number):
        super().__init__('testbed_robot_%d'%robot_number)
        self.robot_number = _robot_number
        self.odometry_message_number = 0
        self.image_message_number = 0
        self.odometry_publisher = self.create_publisher(String, 'odometry')
        self.image_publisher = self.create_publisher(String, 'image')
        odometry_timer_period = 0.1 # second
        image_timer_period = 0.1
        self._odometry_timer = self.create_timer(odometry_timer_period,
                                                 self.odometry_timer_callback)
        self._image_timer = self.create_timer(image_timer_period,
                                                 self.image_timer_callback)

    def odometry_timer_callback(self):
        self.odometry_message_number += 1
        msg = String()
        msg.data = 'r%d Odometry value %d'%(
                        self.robot_number, self.odometry_message_number)
        self.get_logger().info('Publishing odometry %d'%
                               self.odometry_message_number)
        self.odometry_publisher.publish(msg)

    def image_timer_callback(self):
        self.image_message_number += 1
        image_data = "%s %s"%(self.image_message_number, "a"*IMAGE_LENGTH)
        msg = String()
        msg.data = 'r%d Image %d value %s'%(self.robot_number,
                                            self.image_message_number,
                                            "a"*IMAGE_LENGTH)
        self.get_logger().info('Publishing image message %d size %d'%(
                               self.image_message_number, len(msg.data)))
        self.image_publisher.publish(msg)

def main():
    parser = ArgumentParser(description="Testbed robot.")
    parser.add_argument("robot_number", type=int, help="The robot number.")
    parser.add_argument("-n", "--use_nns", action="store_true",
                        help="Run in its own Network Namespace")
    args = parser.parse_args()

    if args.use_nns:
        set_nns("nns%d"%(args.robot_number+1))

    rclpy.init()
    node = R(args.robot_number)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Do not invoke directly, invoke from ros2.")
    main()

