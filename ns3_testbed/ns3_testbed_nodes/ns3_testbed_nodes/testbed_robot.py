#!/usr/bin/env python3
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from os.path import join, expanduser
from collections import defaultdict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.testbed_codec import testbed_decode
from ns3_testbed_nodes.pipe_logger import PipeLogger, PIPE_NAME
from ns3_testbed_nodes.setup_reader import read_setup

class TestbedRobot(Node):

    def _make_publisher_timer_callback_function(self, subscription_name, size):
        def fn(self):
            self.get_logger().info("publisher callback for %s"%subscription_name)
            self.counters[subscription_name] += 1
            count = self.counters[subscription_name]
            msg = String()
            msg.data = testbed_encode(self.robot_name,
                                      subscription_name,
                                      count,
                                      size)
            self.publisher_managers.publish(msg)
        return fn

    def _subscription_callback_function(self, msg):
        self.get_logger().info("subscription callback")
        source, name, number, size, dt = testbed_decode(msg.data)
        response = "%s,%s,%d,%d,%f"%(source, name, number, size, dt)
        if self.pipe_logger:
            self.pipe_logger.log(response)
        else:
            # use Node's native logger
            self.get_logger().info(response)

    def __init__(self, robot_name, publishers, subscribers, use_pipe_logger):
        super().__init__(robot_name)
        self.pipe_logger = PipeLogger()
        self.robot_name = robot_name
        if use_pipe_logger:
            self.pipe_logger = PipeLogger()
        else:
            self.pipe_logger = None

        # start publishers
        self.counters = defaultdict(int)
        self.publisher_managers = dict()
        for publisher in publishers:
            # only publish to subscriptions intended for this robot
            if publisher.robot_name != robot_name:
                continue

            # the publisher parameters
            period = 1/publisher.frequency
            subscription_name = publisher.subscription_name
            size = publisher.size

            # the publisher manager
            publisher_manager = self.create_publisher(String, subscription_name)
            self.publisher_managers[subscription_name] = publisher_manager

            # the counter
            counter = self.counters[subscription_name]

            # the callback function that is dynamically created using closure
            publisher_timer_callback_function = \
                            self._make_publisher_timer_callback_function(
                                                   subscription_name, size)

        # start subscribers
        subscriptions = list()
        for subscriber in subscribers:

            # only subscribe to subscriptions intended for this robot
            if subscriber.robot_name != robot_name:
                continue

            subscription = self.create_subscription(
                                       String,
                                       subscriber.subscription_name,
                                       self._subscription_callback_function)
            subscriptions.append(subscription)

def main():
    default_setup_file = join(expanduser("~"),
                         "gits/ns3_gazebo/ns3_testbed/csv_setup/example1.csv")
    parser = ArgumentParser(description="Generic ns-3 testbed robot.",
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("robot_name", type=str,
                        help="The name of this robot node.")
    parser.add_argument("-p","--use_pipe_logger", action="store_true",
                        help="Send output to pipe '%s'"%PIPE_NAME)
    parser.add_argument("-s","--setup_file", type=str,
                        help="The CSV setup file.",
                        default = default_setup_file)
    args = parser.parse_args()
    print("Starting testbed_robot %s"%args.robot_name)

    # get setup parameters
    publishers, subscribers = read_setup(args.setup_file)

    rclpy.init()
    robot_node = TestbedRobot(args.robot_name, publishers, subscribers,
                              args.use_pipe_logger)
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

