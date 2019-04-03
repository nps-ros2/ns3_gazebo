// Adapted from https://github.com/ros-planning/navigation2/blob/master/nav2_robot/src/robot.cpp

#include "diff_drive_robot.hpp"

#include <string>
#include <exception>

namespace diff_drive_robot {

DiffDriveRobot::DiffDriveRobot(rclcpp::Node::SharedPtr & _rclcpp_node,
                               ns3::NodeContainer* _ns3_nodes):
            rclcpp_node(_rclcpp_node), ns3_nodes(_ns3_nodes)
{
  odom_sub_ = rclcpp_node->create_subscription<nav_msgs::msg::Odometry>(
    "/demo/odom_demo", std::bind(&DiffDriveRobot::onOdomReceived, this, std::placeholders::_1));
}

void
DiffDriveRobot::onOdomReceived(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // Move ns-3's node 0 antenna location
  ns3::Ptr<ns3::Node> node = ns3_nodes->Get(0);
  ns3::Ptr<ns3::ConstantPositionMobilityModel> mobility_model =
                        node->GetObject<ns3::ConstantPositionMobilityModel>();
  auto vector = mobility_model->GetPosition();
  vector.x = msg->pose.pose.position.x;
  mobility_model->SetPosition(vector);
  std::stringstream ss;
  ss << "Set Node 0 antenna x: " << vector.x;
//  std::cout << ss.str() << std::endl;
  RCLCPP_INFO(rclcpp_node->get_logger(), ss.str().c_str());
}

std::string
DiffDriveRobot::getName()
{
  return "diff_drive_robot";
}

}  // namespace

