#!/usr/bin/env python3
from argparse import ArgumentParser
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.testbed_codec import testbed_encode

#IMAGE_LENGTH=640*480*1
IMAGE_LENGTH=5
ODOMETRY_LENGTH=150

class R(Node):

    def __init__(self, _robot_number):
        super().__init__('testbed_robot_%d'%_robot_number)
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
#        print("odometry_timer_callback")
        self.odometry_message_number += 1
        msg = String()
        msg.data = testbed_encode("R%d"%self.robot_number, "odometry", 
                         self.odometry_message_number, ODOMETRY_LENGTH)

        self.get_logger().info('Publishing odometry message number %d'%
                               self.odometry_message_number)
#        print('Publishing odometry %d'% self.odometry_message_number)
        self.odometry_publisher.publish(msg)

    def image_timer_callback(self):
        self.image_message_number += 1
        msg = String()
        msg.data = testbed_encode("R%d"%self.robot_number, "image",
                         self.image_message_number, IMAGE_LENGTH)
#        self.get_logger().info('Publishing image message number %d size %d'%(
#                               self.image_message_number, len(msg.data)))
        self.image_publisher.publish(msg)

def main():
    parser = ArgumentParser(description="Testbed robot.")
    parser.add_argument("robot_number", type=int, help="The robot number.")
    args = parser.parse_args()

    rclpy.init()
    node = R(args.robot_number)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("Do not invoke directly, invoke from ros2.")
    main()

