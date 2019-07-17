
#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include <chrono> // for timer
#include <functional> // for bind
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "testbed_robot.hpp"

// publisher_callback
publisher_callback_t::publisher_callback_t(testbed_robot_t* _r_ptr,
                       const std::string _subscription_name,
                       const unsigned int _size,
                       const unsigned int _frequency,
                       const bool _verbose) :
           count(0),
           msg(),
           publisher(),

           r_ptr(_r_ptr),
           subscription_name(_subscription_name),
           size(_size),
           frequency(_frequency),
//zz           timer(_r_ptr->create_wall_timer(std::chrono::seconds(1/_frequency),
//zz                                           this->publish_message)),
           verbose(_verbose),
           node_logger(_r_ptr->get_logger()) {
}

void publisher_callback_t::publish_message() {

  msg->data = "zzzz";
  if (verbose) {
    RCLCPP_INFO(node_logger, "Publishing: '%s'", msg->data.c_str());
  }

  publisher->publish(msg);
}

// subscriber_callback
//typedef void(subscriber_callback_t::*subscriber_callback_t)(
//                           const std_msgs::msg::String::SharedPtr msg);
subscriber_callback_t::subscriber_callback_t(testbed_robot_t* _r_ptr,
                      const std::string _subscription_name,
                      const bool _no_pipe,
                      const bool _verbose) : //zz also pipe_logger
         r_ptr(_r_ptr),
         subscription(_r_ptr->create_subscription<std_msgs::msg::String>(
                   _subscription_name,
                   &subscriber_callback_t::subscriber_callback)),
//                           std_msgs::msg::String::SharedPtr msg)
//),
//                   std::bind(&subscriber_callback_t::subscriber_callback,
//                             std::placeholders::_1))),
         no_pipe(_no_pipe),
         verbose(_verbose),
         node_logger(_r_ptr->get_logger()) {
}

void subscriber_callback_t::subscriber_callback(
                           const std_msgs::msg::String::SharedPtr msg) {
  if(verbose) {
    RCLCPP_INFO(node_logger, "subscription callback: '%s'",
                 msg->data.c_str());
  }

  if(no_pipe) {
    // use node_logger
  } else {
    // zz use pipe
  }
}

