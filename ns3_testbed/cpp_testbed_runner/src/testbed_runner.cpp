#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <getopt.h>

#include "rclcpp/rclcpp.hpp"
//#include "rcutils/cmdline_parser.h"

#include "std_msgs/msg/string.hpp"

//#include "setup_reader.hpp"
#include "testbed_robot.hpp"

static unsigned int count = 5;
static const std::string default_setup_file = "../../csv_setup/example1.csv";
static std::string setup_file = default_setup_file;
static bool no_nns = true;
static bool no_pipe = true;
static bool verbose = false;

void print_usage()
{
  printf("Usage:\n");
  printf("testbed_runner -h|-H|-c <count>|--count <count>|-s <file>|--setup_file <file>|-n|--no_nns|-p|--no_pipe|-v|--verbose\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-c <count>: Number of robots, starting at 1.\n");
  printf("-s <setup file>: The CSV setup file.\n");
  printf("-n: No network namespace.\n");
  printf("-p: No pipe.\n");
  printf("-v: verbose.\n");
}

// parse user input
void get_options(int argc, char *argv[]) {

  // parse options
  int option_index; // not used
  while (1) {

    const struct option long_options[] = {
      // options
      {"help",                          no_argument, 0, 'h'},
      {"Help",                          no_argument, 0, 'H'},
      {"count",                   required_argument, 0, 'c'},
      {"setup_file",              required_argument, 0, 's'},
      {"no_nns",                  required_argument, 0, 'n'},
      {"no_pipe",                 required_argument, 0, 'p'},
      {"verbose",                       no_argument, 0, 'v'},

      // end
      {0,0,0,0}
    };

    int ch = getopt_long(argc, argv, "hHc:s:npv", long_options, &option_index);

    if (ch == -1) {
      // no more arguments
      break;
    }
    if (ch == 0) {
      // command options set flags and use ch==0
      continue;
    }
    switch (ch) {
      case 'h': {	// help
        print_usage();
        exit(0);
      }
      case 'H': {	// Help
        print_usage();
        exit(0);
      }
      case 'c': {	// count
        count = std::atoi(optarg);
        break;
      }
      case 's': {	// setup_file
        setup_file = optarg;
        break;
      }
      case 'n': {	// no nns
        no_nns = false;
        break;
      }
      case 'p': {	// no pipe
        no_pipe = false;
        break;
      }
      case 'v': {	// count
        verbose = true;
        break;
      }
      default:
        std::cerr << "unexpected command character " << ch << "\n"
                  << "See usage.\n";
        exit(1);
    }
  }

//  // parse the remaining tokens that were not consumed by options
//  argc -= optind;
//  argv += optind;

}

// start, stay here until done.
void start_robots(unsigned int count, bool no_nns, bool no_pipe,
                         bool verbose, publishers_subscribers_t* ps_ptr) {

  std::vector<std::thread*> threads;

  for(unsigned int i = 1; i<= count; i++) {
    std::cout << "Starting testbed runner " << i << std::endl;

    // nns#, R#
    std::stringstream ss1;
    std::stringstream ss2;
    ss1 << "nns" << i;
    ss2 << "R" << i;
    std::string nns = ss1.str();
    std::string r = ss2.str();

    std::cout << "Starting " << nns << " " << r << std::endl;
    threads.push_back(new std::thread(testbed_robot_run, nns, r,
                                      no_nns, no_pipe, verbose, ps_ptr));
  }
  for (std::vector<std::thread*>::iterator it = threads.begin();
       it != threads.end(); ++it) {
    (*it)->join();
  }
}


int main(int argc, char * argv[])
{
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  get_options(argc, argv);

  // pointer to static list of publishers and subscribers
  publishers_subscribers_t* publishers_subscribers_ptr =
                      new publishers_subscribers_t(setup_file, verbose);

  std::cout << "num publisers: " << publishers_subscribers_ptr->publishers.size() << std::endl;
  std::cout << "num subscribers: " << publishers_subscribers_ptr->subscribers.size() << std::endl;

  // call once per process for rclcpp
  rclcpp::init(argc, argv);

  start_robots(count, no_nns, no_pipe, verbose, publishers_subscribers_ptr);
  std::cout << "Running..." << std::endl;

  rclcpp::shutdown();
  return 0;
}
