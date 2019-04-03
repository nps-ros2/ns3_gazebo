// adapted from https://raw.githubusercontent.com/ros-planning/navigation2/master/nav2_robot/include/nav2_robot/robot.hpp

#ifndef DIFF_DRIVE_ROBOT_HPP
#define DIFF_DRIVE_ROBOT_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"
#include "nav_msgs/msg/odometry.hpp"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/tap-bridge-module.h"

namespace diff_drive_robot {

class DiffDriveRobot {
public:
  explicit DiffDriveRobot(rclcpp::Node::SharedPtr & _rclcpp_node,
                          ns3::NodeContainer* _ns3_nodes);
  DiffDriveRobot() = delete;

  std::string getName();

protected:
  // The ROS node to use to create publishers and subscribers
  rclcpp::Node::SharedPtr rclcpp_node;

  // The NS-3 nodes
  ns3::NodeContainer* ns3_nodes;

  // Subscription
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // Subscription callback
  void onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg);
};

}  // namespace

#endif
