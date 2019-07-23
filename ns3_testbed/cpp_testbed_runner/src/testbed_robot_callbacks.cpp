
#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include <sstream>
#include <chrono> // for timer
#include <functional> // for bind
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
//#include "testbed_message/testbed_message.hpp"
#include "cpp_testbed_runner/msg/testbed_message.hpp"
//#include "testbed_message.hpp"

#include "testbed_robot.hpp"

using float_sec = std::chrono::duration<float>;

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
           publisher(_r_ptr->create_publisher<std_msgs::msg::String>(
                                          _subscription_name, _qos_profile)),
           timer(_r_ptr->create_wall_timer(std::chrono::microseconds(1000000/_frequency),
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
}

using Pstd_mem = void(subscriber_callback_t::*)
                     (std_msgs::msg::String::SharedPtr);

// subscriber_callback
//typedef void(subscriber_callback_t::*subscriber_callback_t)(
//                           const std_msgs::msg::String::SharedPtr msg);
subscriber_callback_t::subscriber_callback_t(testbed_robot_t* _r_ptr,
                      const std::string _subscription_name,
                      const rmw_qos_profile_t _qos_profile,
                      const bool _no_pipe,
                      const bool _verbose) : //zz also pipe_logger
         r_ptr(_r_ptr),
         subscription_name(_subscription_name),

         subscription(0),

//         subscription(_r_ptr->create_subscription<std_msgs::msg::String>(
//                   _subscription_name,
//
//                   &subscriber_callback_t::subscriber_callback
//
////                   this->*void (&subscriber_callback_t::subscriber_callback)
////                        (std_msgs::msg::String::SharedPtr)
//
//)),

//
//void(this->&subscriber_callback_t::subscriber_callback)(
//                           std_msgs::msg::String::SharedPtr))),


////                   this.subscriber_callback_t::subscriber_callback
//                   void(subscriber_callback_t::* ptfptr
//                   &(subscriber_callback_t::subscriber_callback)(
//                           const std_msgs::msg::String::SharedPtr msg))
//
//)),
////                           std_msgs::msg::String::SharedPtr msg)
////),
////                   std::bind(&subscriber_callback_t::subscriber_callback,
////                             std::placeholders::_1))),

         qos_profile(_qos_profile),
         no_pipe(_no_pipe),
         verbose(_verbose),
         node_logger(r_ptr->get_logger()) {

//  void (&subscriber_callback_t::subscriber_callback)
//       (std_msgs::msg::String::SharedPtr) f =
//                          &subscriber_callback_t::subscriber_callback;
//  f("zz");


  auto callback =
          [this](const std_msgs::msg::String::SharedPtr msg) -> void
          {
            std::cerr << "subscriber.zzzzzzzzzzzzzz\n";
            this->subscriber_callback(msg);
          };
  subscription = _r_ptr->create_subscription<std_msgs::msg::String>(
                      subscription_name, callback);


//  void (subscriber_callback_t::* p)(std_msgs::msg::String::SharedPtr) =
//                             &this.subscriber_callback_t::subscriber_callback;
//
//  subscription = _r_ptr->create_subscription<std_msgs::msg::String>(
//                      subscription_name,
//                      this.*p
//                 );

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

