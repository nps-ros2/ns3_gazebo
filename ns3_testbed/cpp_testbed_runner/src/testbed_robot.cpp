#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "testbed_robot.hpp"
#include "setup_reader.hpp"
#include "testbed_robot_callbacks.hpp"


testbed_robot_t::testbed_robot_t(
                        const std::string& _nns,
                        const std::string& _r,
                        bool _no_pipe, bool _verbose,
                        const publishers_subscribers_t* const _ps_ptr) :
          Node("testbed_robot_"+_r),
          nns(_nns), r(_r), no_pipe(_no_pipe), verbose(_verbose),
          ps_ptr(_ps_ptr) {

//  std::cout << "testbed_robot_t.a" << std::endl;
  // publishers
  for (std::vector<publish_record_t>::const_iterator it =
                 ps_ptr->publishers.begin();
                 it != ps_ptr->publishers.end(); ++it) {

//    std::cout << "testbed_robot_t.b" << std::endl;

    // not meant for this robot
    if(it->robot_name != r) {
      continue;
    }

    std::cout << "testbed_robot_t.c" << std::endl;

    // add publisher
    publisher_callbacks.emplace_back(new publisher_callback_t(this,
                                      it->subscription,
                                      it->size, it->frequency,
                                      it->qos_profile,
                                      verbose));
  }

  std::cout << "testbed_robot_t.d" << std::endl;
  // subscribers
  for (std::vector<subscribe_record_t>::const_iterator it =
       ps_ptr->subscribers.begin();
       it != ps_ptr->subscribers.end(); ++it) {

    std::cout << "testbed_robot_t.e" << std::endl;

    // not meant for this robot
    if(it->robot_name != r) {
      continue;
    }

    // add subscriber
    std::cout << "testbed_robot_t.f" << std::endl;
    subscriber_callbacks.emplace_back(new subscriber_callback_t(this,
                       it->subscription, it->qos_profile, no_pipe, verbose));
  }
  std::cout << "testbed_robot_t.g" << std::endl;
}

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

  std::cout << "testbed_robot_run.a" << std::endl;
  auto node = std::make_shared<testbed_robot_t>(nns, r, no_pipe,
                                                verbose, ps_ptr);
  std::cout << "testbed_robot_run.b" << std::endl;
  rclcpp::spin(node); // block until Ctrl-C
}

