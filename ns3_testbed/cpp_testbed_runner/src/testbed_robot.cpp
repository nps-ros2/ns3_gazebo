#include <vector>
#include <memory> // for shared_ptr
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "testbed_robot.hpp"
#include "setup_reader.hpp"
#include "testbed_robot_callbacks.hpp"
#include "pipe_writer.hpp"
#include "set_nns.hpp"


testbed_robot_t::testbed_robot_t(
                        const std::string& _nns,
                        const std::string& _r,
                        bool _use_pipe, bool _verbose,
                        const publishers_subscribers_t* const _ps_ptr) :
          Node("testbed_robot_"+_r),
          nns(_nns), r(_r), use_pipe(_use_pipe),
          pipe_writer(_use_pipe),
          verbose(_verbose),
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
    publisher_callbacks.emplace_back(new publisher_callback_t(this,
                                      it->subscription,
                                      it->size, it->microseconds,
                                      it->qos_profile,
                                      verbose));
  }

  // subscribers
  for (std::vector<subscribe_record_t>::const_iterator it =
       ps_ptr->subscribers.begin();
       it != ps_ptr->subscribers.end(); ++it) {

    // not meant for this robot
    if(it->robot_name != r) {
      continue;
    }

    // add subscriber
    subscriber_callbacks.emplace_back(new subscriber_callback_t(this,
                       it->subscription, it->qos_profile, use_pipe, verbose));
  }
}

// entry function to start a testbed robot
void testbed_robot_run(std::string nns, std::string r,
                       bool use_nns, bool use_pipe, bool verbose,
                       publishers_subscribers_t* ps_ptr) {

  // maybe move to nns
  if(use_nns) {
    set_nns(nns);
  }

  auto node = std::make_shared<testbed_robot_t>(nns, r, use_pipe,
                                                verbose, ps_ptr);

  rclcpp::spin(node); // block until Ctrl-C

}

