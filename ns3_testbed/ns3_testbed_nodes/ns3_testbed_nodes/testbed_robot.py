#!/usr/bin/env python3
from sys import stdout
from argparse import ArgumentParser, ArgumentDefaultsHelpFormatter
from os.path import join, expanduser
from collections import defaultdict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from ns3_testbed_nodes.testbed_codec import testbed_encode, testbed_decode
from ns3_testbed_nodes.pipe_logger import PipeLogger, PIPE_NAME
from ns3_testbed_nodes.setup_reader import read_setup

class TestbedRobot(Node):

    def _make_publisher_timer_callback_function(self, subscription_name, size):
        def fn():
            if self.verbose:
                self.get_logger().info("publisher callback for %s"%subscription_name)
            self.counters[subscription_name] += 1
            count = self.counters[subscription_name]
            msg = String()
            msg.data = testbed_encode(self.robot_name,
                                      subscription_name,
                                      count,
                                      size)
#            self.get_logger().info("_make_publish: '%s'"%msg.data[:60])
            self.publisher_managers[subscription_name].publish(msg)
        return fn

    def _subscription_callback_function(self, msg):
#        self.get_logger().info("_make_subscribe: '%s'"%msg.data[:60])
        if self.verbose:
            self.get_logger().info("subscription callback")
        source, name, number, size, dt = testbed_decode(msg.data)
        response = "%s,%s,%d,%d,%f"%(source, name, number, size, dt)

        if self.pipe_logger:
            self.pipe_logger.log(response)
        else:
            # use Node's native logger
            self.get_logger().info(response)

    def __init__(self, robot_name, publishers, subscribers, no_pipe, verbose):
        super().__init__(robot_name)
        self.robot_name = robot_name
        self.verbose = verbose
        if no_pipe:
            self.pipe_logger = None
        else:
            self.pipe_logger = PipeLogger()

        # start publishers
        self.counters = defaultdict(int)
        self.publisher_managers = dict()
        self.timers = list()
        for publisher in publishers:
            # only publish to subscriptions intended for this robot
            if publisher.node != robot_name:
                continue

            # the publisher parameters
            period = 1/publisher.frequency
            subscription_name = publisher.subscription
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
            timer = self.create_timer(period, publisher_timer_callback_function)
            self.timers.append(timer)

        # start subscribers
        subscriptions = list()
        for subscriber in subscribers:

            # only subscribe to subscriptions intended for this robot
            if subscriber.node != robot_name:
                continue

            subscription = self.create_subscription(
                                       String,
                                       subscriber.subscription,
                                       self._subscription_callback_function)
            subscriptions.append(subscription)

def main():
    default_setup_file = join(expanduser("~"),
                         "gits/ns3_gazebo/ns3_testbed/csv_setup/example1.csv")
    parser = ArgumentParser(description="Generic ns-3 testbed robot.",
                            formatter_class=ArgumentDefaultsHelpFormatter)
    parser.add_argument("robot_name", type=str,
                        help="The name of this robot node.")
    parser.add_argument("-s","--setup_file", type=str,
                        help="The CSV setup file.",
                        default = default_setup_file)
    parser.add_argument("-p","--no_pipe", action="store_true",
                        help="Do not send output to pipe '%s'."%PIPE_NAME)
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable verbose diagnostics.")
    args = parser.parse_args()
    print("Starting testbed_robot %s"%args.robot_name)
    stdout.flush()

    # get setup parameters
    publishers, subscribers = read_setup(args.setup_file, args.verbose)
    stdout.flush()

    rclpy.init()
    robot_node = TestbedRobot(args.robot_name, publishers, subscribers,
                              args.no_pipe, args.verbose)
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

