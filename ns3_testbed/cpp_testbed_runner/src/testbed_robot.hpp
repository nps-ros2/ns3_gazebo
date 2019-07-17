#ifndef TESTBED_ROBOT_HPP
#define TESTBED_ROBOT_HPP

#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "setup_reader.hpp"
#include "testbed_robot_callbacks.hpp"

class testbed_robot_t : public rclcpp::Node {

  private:
  const std::string nns;
  const std::string r;
  const bool no_pipe;
  const bool verbose;
  const publishers_subscribers_t* ps_ptr;
  std::vector<publisher_callback_t> publisher_callbacks;
  std::vector<subscriber_callback_t> subscriber_callbacks;

  public:
  explicit testbed_robot_t(const std::string& _nns, const std::string& _r,
                        bool _no_pipe, bool _verbose,
                        const publishers_subscribers_t* const _ps_ptr) :
          Node("testbed_robot"),
          nns(_nns), r(_r), no_pipe(_no_pipe), verbose(_verbose),
          ps_ptr(_ps_ptr) {

    // publishers
    for (std::vector<publish_record_t>::const_iterator it =
                 ps_ptr->publishers.begin();
                 it != ps_ptr->publishers.end(); ++it) {

      // not meant for this robot
      if(it->robot_name != r) {
        continue;
      }

      // add publisher
      publisher_callbacks.emplace_back(publisher_callback_t(this,
                                        it->subscription,
                                        it->size, it->frequency, verbose));
    }

    // subscribers
    for (std::vector<subscribe_record_t>::const_iterator it =
         ps_ptr->subscribers.begin();
         it != ps_ptr->subscribers.end(); ++it) {

      // not meant for this robot
      if(it->robot_name != r) {
        continue;
      }

      // add subscribe
      subscriber_callbacks.emplace_back(subscriber_callback_t(this,
                                  it->subscription, no_pipe, verbose));
    }
  }
};

// entry function to start a testbed robot
void testbed_robot_run(std::string nns, std::string r,
                       bool no_nns, bool no_pipe, bool verbose,
                       publishers_subscribers_t* ps_ptr) {

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

  auto node = std::make_shared<testbed_robot_t>(nns, r, no_pipe,
                                                verbose, ps_ptr);
  rclcpp::spin(node); // block until Ctrl-C
}

#endif
