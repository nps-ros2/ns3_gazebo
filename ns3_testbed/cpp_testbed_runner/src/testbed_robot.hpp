#ifndef TESTBED_ROBOT_HPP
#define TESTBED_ROBOT_HPP

#include "setup_reader.hpp"

void testbed_robot_run(std::string nns, std::string r,
                       bool no_nns, bool no_pipe, bool verbose,
                       publishers_subscribers_t* ps_ptr) {

  // call once per process for rclcpp
  rclcpp::init(argc, argv);

  auto node = std::make_shared<

  // maybe move to nns
  if(!no_nns) {
    // set nns
    // zz
  }

  // maybe start pipe
  if(!no_pipe) {
    // open pipe
    // zz
  }


class publisher_callback_t {
  private:
  int count;
  std::shared_ptr<std_msgs::msg::String> msg;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::TimerBase::ShardPtr timer;
  const std::string subscription_name;
  const unsigned int size;

  public:
  publisher_callback_t(const std::string _subscription_name,
                       const unsigned int _size, const unsigned int frequency,
                       const bool _verbose) :
           count(0),
           msg(std::make_shared<std_msgs::msg::String>()),
           publisher(rclcpp::Publisher<std_msgs::msg::String>::SharedPtr),
           timer(this->create_wall_timer(1/frequency, publish_message);
           subscriptioin_name(_subscription_name),
           size(_size),
           verbose(_verbose) {
  }

  void publish_message() {

    msg->data = "zzzz";
    if (verbose) {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());
    }

    publisher->publish(msg);
  }
};

class listener_callback_t {
  private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;

  public:
  listener_callback_t(const std::string _subscription_name,
                      const bool _verbose) : //zz also pipe_logger
         subscription(rclcpp::Subscription<std_msgs::msg::String>::SharedPtr),
         verbose(_verbose) {
  }

  void listener_callback(const std::msgs::msg::String::SharedPtr msg) {
    if(verbose) {
      RCLCPP_INFO(this->get_logger(), "subscription callback: '%s'",
                   msg->data.c_str());
    }

    if(no_pipe) {
      // use native logger
    } else {
      // zz use pipe
    }
  }
};


class testbed_robot_t : public rclcpp::Node {

  const std::string nns;
  const std::string r;
  const bool no_pipe;
  const bool verbose;
  const ps_ptr;
  std::vector<publisher_callback_t> publishers;
  std::vector<subscriber_callback_t> subscribers;

  public:
  explicit testbed_robot_t(const std::string& nns, const std::string& r,
                        bool no_pipe, bool verbose,
                        const publishers_subscribers_t* const ps_ptr) :
          nns(nns), r(r), no_pipe(no_pipe), verbose(verbose), ps_ptr(ps_ptr) {

    // publishers
    for (std::vector<publish_record_t>::iterator it = ps_ptr.publishers.begin();
         it != ps_ptr.publishers.end(); ++it) {

      // not meant for this robot
      if(it->node != r) {
        continue;
      }

      // add publisher
      publishers.emplace_back(it->subscription, it->size,
                              it->frequency, verbose);
    }

    // subscribers
    for (std::vector<subscribe_record_t>::iterator it =
         ps_ptr.subscribers.begin();
         it != ps_ptr.subscribers.end(); ++it) {

      // not meant for this robot
      if(it->node != r) {
        continue;
      }

      // add subscribe
      subscribers.emplace_back(it->subscription, verbose);
    }
  }
};

