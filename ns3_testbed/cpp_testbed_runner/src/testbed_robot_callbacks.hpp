#ifndef TESTBED_ROBOT_CALLBACKS_HPP
#define TESTBED_ROBOT_CALLBACKS_HPP

#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include "rclcpp/rclcpp.hpp"

class testbed_robot_t;

class publisher_callback_t {
  private:
  int count;
  std::shared_ptr<std_msgs::msg::String> msg;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;

  testbed_robot_t* r_ptr;
  const std::string subscription_name;
  const unsigned int size;
  const unsigned int frequency;
  rclcpp::TimerBase::SharedPtr timer;
  const bool verbose;
  rclcpp::Logger node_logger;

  public:
  publisher_callback_t(testbed_robot_t* _r_ptr,
                       const std::string _subscription_name,
                       const unsigned int _size,
                       const unsigned int _frequency,
                       const bool _verbose);
  void publish_message();
};

class subscriber_callback_t {
  private:
  testbed_robot_t* r_ptr;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  const bool no_pipe;
  const bool verbose;
  rclcpp::Logger node_logger;

  public:
  subscriber_callback_t(testbed_robot_t* _r_ptr,
                        const std::string _subscription_name,
                        const bool _no_pipe,
                        const bool _verbose); //zz also pipe_logger

  void subscriber_callback(std_msgs::msg::String::SharedPtr msg);
};

#endif
