#ifndef START_ROBOTS_HPP
#define START_ROBOTS_HPP

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"

//using namespace std::chrono_literals;


inline void start_robots(unsigned int count, bool no_nns, bool no_pipe,
                         bool verbose, publishers_subscribers_t* ps_ptr) {

  std::vector<std::thread> threads;

  for(unsigned int i = 1; i<= count; i++) {
    std::cout << "Starting testbed runner " << i << std::eoln;

    // nns#, R#
    std::stringstream ss1;
    std::stringstream ss2;
    ss1 << "nns" << i;
    ss2 << "R" << i;
    std::string nns = ss1.str();
    std::string r = ss2.str();

    std::cout << "Starting " << nns << " " << r << std::eoln;
    std::thread testbed_robot_run














      for i in range(1,args.count+1):
        print("Starting testbed runner %d ..."%i)
        nns="nns%d"%i
        r="R%d"%(i)
        print("start %s %s..."%(nns,r))
        nns_start(r, nns,
                  ["ros2","run","ns3_testbed_nodes", "testbed_robot", r])
    print("Running...")














CLONE_NEWNET = 0x40000000

def errcheck(ret, func, args):
    if ret == -1:
        e = get_errno()
        raise OSError(e, os.strerror(e))

libc = CDLL('libc.so.6', use_errno=True)
libc.setns.errcheck=errcheck

"""Set NNS or raise ValueError.  See libc.setns."""
def set_nns(nns_name):
    print("set_nns %s..."%nns_name)
    nnspath = "/var/run/netns/%s"%nns_name
    if not os.path.exists(nnspath):
        error = "Error: NNS %s is not defined.  " \
                "Please run the setup script."%nnspath
        print(error)
        raise ValueError(error)
    with open(nnspath) as fd:
        if hasattr(fd, 'fileno'):
            print("hasattr fileno.")
            fd = fd.fileno()
        status = libc.setns(fd, CLONE_NEWNET)
        if status:
            error = "Error: failure with %s"%nnspath
            print(error)
            raise ValueError(error)
        else:
            print("set_nns %s Done."%nns_name)













    # --setup_file
    cmd.append("-s")
    cmd.append(args.setup_file)

    # --no_nns
    if args.no_nns:
        # suppress nns
        pass
    else:
        # default uses nns
        try:
            set_nns(nns)
        except PermissionError as e:
            print("Error: This program must be run from root when using "
                  "network namespaces.\n"
                  "Use '--no_nns' to run outside namespaces.\nAborting.")
            exit(1)

    # --no_pipe
    if args.no_pipe:
        cmd.append("-p")

    # --verbose
    if args.verbose:
        cmd.append("-v")

    # start
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    t = threading.Thread(target=output_handler, args=(p, name))
    t.start()

























































// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node
{
public:
  explicit Talker(const std::string & topic_name)
  : Node("talker")
  {
    msg_ = std::make_shared<std_msgs::msg::String>();

    // Create a function for when messages are to be sent.
    auto publish_message =
      [this]() -> void
      {
        msg_->data = "Hello World: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg_->data.c_str());

        // Put the message into a queue to be processed by the middleware.
        // This call is non-blocking.
        pub_->publish(msg_);
      };

    // Create a publisher with a custom Quality of Service profile.
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name, custom_qos_profile);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_ = 1;
  std::shared_ptr<std_msgs::msg::String> msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    print_usage();
    return 0;
  }

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Parse the command line options.
  auto topic = std::string("chatter");
  char * cli_option = rcutils_cli_get_option(argv, argv + argc, "-t");
  if (nullptr != cli_option) {
    topic = std::string(cli_option);
  }

  // Create a node.
  auto node = std::make_shared<Talker>(topic);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
