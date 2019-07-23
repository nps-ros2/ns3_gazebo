#ifndef TESTBED_ROBOT_HPP
#define TESTBED_ROBOT_HPP

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "testbed_robot_callbacks.hpp"
#include "setup_reader.hpp"

class testbed_robot_t : public rclcpp::Node {

  private:
  const std::string nns;
  const std::string r;
  const bool no_pipe;
  const bool verbose;
  const publishers_subscribers_t* ps_ptr;
  std::vector<publisher_callback_t*> publisher_callbacks;
  std::vector<subscriber_callback_t*> subscriber_callbacks;

  public:
  explicit testbed_robot_t(const std::string& _nns, const std::string& _r,
                        bool _no_pipe, bool _verbose,
                        const publishers_subscribers_t* const _ps_ptr);
};

// entry function to start a testbed robot
void testbed_robot_run(std::string nns, std::string r,
                       bool no_nns, bool no_pipe, bool verbose,
                       publishers_subscribers_t* ps_ptr);

#endif

