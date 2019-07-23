
#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include <sstream>
#include <chrono> // for timer
#include <functional> // for bind
#include "rclcpp/rclcpp.hpp"
//#include "rclcpp/time.hpp"
// https://discourse.ros.org/t/ros2-how-to-use-custom-message-in-project-where-its-declared/2071
#include "cpp_testbed_runner/msg/testbed_message.hpp"

#include "testbed_robot.hpp"

// publisher_callback
publisher_callback_t::publisher_callback_t(testbed_robot_t* _r_ptr,
                       const std::string _subscription_name,
                       const unsigned int _size,
                       const unsigned int _frequency,
                       const rmw_qos_profile_t _qos_profile,
                       const bool _verbose) :
           r_ptr(_r_ptr),
           subscription_name(_subscription_name),
           size(_size),
           frequency(_frequency),
           qos_profile(_qos_profile),
           verbose(_verbose),

           count(0),
           publisher(_r_ptr->create_publisher<
                     cpp_testbed_runner::msg::TestbedMessage>(
                     _subscription_name, _qos_profile)),
           timer(_r_ptr->create_wall_timer(
                     std::chrono::microseconds(1000000/_frequency),
                     std::bind(&publisher_callback_t::publish_message, this))),
           node_logger(r_ptr->get_logger()) 
{
  std::cerr << "publisher_callback_t r_ptr: " << r_ptr << "\n";
  std::cerr << "publisher_callback_t subscription: " << subscription_name << "\n";
  std::cerr << "publisher_callback_t this: " << this << "\n";
  std::cerr << "publisher_callback_t size " << size << " frequency "
           << frequency << "\n";
}

// http://www.theconstructsim.com/wp-content/uploads/2019/03/ROS2-IN-5-DAYS-e-book.pdf
void publisher_callback_t::publish_message() {

  std::shared_ptr<cpp_testbed_runner::msg::TestbedMessage> msg(
                std::make_shared<cpp_testbed_runner::msg::TestbedMessage>());

// https://stackoverflow.com/questions/31255486/c-how-do-i-convert-a-stdchronotime-point-to-long-and-back
  std::chrono::time_point<std::chrono::high_resolution_clock> now =
                                std::chrono::high_resolution_clock::now();
  auto now_us = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
  auto epoch = now_us.time_since_epoch();
  long t = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch).count();

  msg->nanoseconds = t;
  msg->source = r_ptr->r;
  msg->message_name = subscription_name;
  msg->message_number = ++count;
  msg->message = std::string(size, subscription_name[0]);

  if (verbose) {
    RCLCPP_INFO(node_logger, "Publishing: %s %d %d",
                             subscription_name, count, size);
  }

  publisher->publish(msg);

/*
  std::shared_ptr<std_msgs::msg::String> msg(std::make_shared<std_msgs::msg::String>());
  msg->data = "zzzz";
  count++;
  std::stringstream ss;
  ss << "size: " << size << ", frequency: " << frequency;
  if (verbose) {
    RCLCPP_INFO(node_logger, "Publishing: '%s'", ss.str().c_str());
    RCLCPP_INFO(node_logger, "Publishing: '%s'", msg->data.c_str());
  }

  publisher->publish(msg);
*/
}

// subscriber_callback
subscriber_callback_t::subscriber_callback_t(testbed_robot_t* _r_ptr,
                      const std::string _subscription_name,
                      const rmw_qos_profile_t _qos_profile,
                      const bool _no_pipe,
                      const bool _verbose) : //zz also pipe_logger
         r_ptr(_r_ptr),
         subscription_name(_subscription_name),

         subscription(0),

         qos_profile(_qos_profile),
         no_pipe(_no_pipe),
         verbose(_verbose),
         node_logger(r_ptr->get_logger()) {


  auto callback =
          [this](const cpp_testbed_runner::msg::TestbedMessage::SharedPtr msg) -> void
          {
            std::cerr << "subscriber.zzzzzzzzzzzzzz\n";
            this->subscriber_callback(msg);
          };
  subscription = _r_ptr->create_subscription<cpp_testbed_runner::msg::TestbedMessage>(
                      subscription_name, callback);



}

void subscriber_callback_t::subscriber_callback(
              const cpp_testbed_runner::msg::TestbedMessage::SharedPtr msg) {
  if(verbose) {
    RCLCPP_INFO(node_logger, "subscription callback: '%s' %d",
                 msg->message_name.c_str(), msg->message_number);
  }

  if(no_pipe) {
    // use node_logger
  } else {
    // zz use pipe
  }
}

/*
  auto callback =
          [this](const std_msgs::msg::String::SharedPtr msg) -> void
          {
            std::cerr << "subscriber.zzzzzzzzzzzzzz\n";
            this->subscriber_callback(msg);
          };
  subscription = _r_ptr->create_subscription<std_msgs::msg::String>(
                      subscription_name, callback);



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
*/

