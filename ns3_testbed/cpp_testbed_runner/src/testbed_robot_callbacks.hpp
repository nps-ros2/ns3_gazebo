#ifndef TESTBED_ROBOT_CALLBACKS_HPP
#define TESTBED_ROBOT_CALLBACKS_HPP

#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "cpp_testbed_runner/msg/testbed_message.hpp"

class testbed_robot_t;

class publisher_callback_t {
  private:
  testbed_robot_t* r_ptr;
  const std::string subscription_name;
  const unsigned int size;
  const unsigned int frequency;
  const rmw_qos_profile_t qos_profile;
  const bool verbose;

  int count;
  rclcpp::Publisher<cpp_testbed_runner::msg::TestbedMessage>::SharedPtr
                                                               publisher;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Logger node_logger;

  public:
  publisher_callback_t(testbed_robot_t* _r_ptr,
                       const std::string _subscription_name,
                       const unsigned int _size,
                       const unsigned int _frequency,
                       const rmw_qos_profile_t _qos_profile,
                       const bool _verbose);
  void publish_message();
};

class subscriber_callback_t {
  private:
  testbed_robot_t* r_ptr;
  const std::string subscription_name;
  rclcpp::Subscription<cpp_testbed_runner::msg::TestbedMessage>::SharedPtr
                                                               subscription;
  const rmw_qos_profile_t qos_profile;
  const bool use_pipe;
  const bool verbose;
  rclcpp::Logger node_logger;

  public:
  subscriber_callback_t(testbed_robot_t* _r_ptr,
                        const std::string _subscription_name,
                        const rmw_qos_profile_t _qos_profile,
                        const bool _use_pipe,
                        const bool _verbose);

  void subscriber_callback(cpp_testbed_runner::msg::TestbedMessage::SharedPtr
                                                               msg);
};

#endif
