#include <iostream>
#include <string>
#include <cassert>
#include <cstdlib> // getenv
#include <sys/types.h>
#include <sys/stat.h> // mkfifo
#include <sys/file.h> // lock
#include <fcntl.h>
#ifndef _GNU_SOURCE
  #define _GNU_SOURCE // setns
#endif
#include <sched.h>  // setns
#include <string.h> // strerror
#include <errno.h>
#include <unistd.h>
#include "set_nns.hpp"

void set_nns(std::string nns_name) {

  std::string nns_path = "/var/run/netns/" + nns_name;

  // nns_path must exist
  struct stat buffer;
  if(stat(nns_path.c_str(), &buffer) != 0) {
    std::cerr << "Error: NNS " << nns_name << " at path " << nns_path << "\n"
              << strerror(errno) << "\n"
              << "Please run the setup script.\n";
    assert(0);
  }

  // open file descriptor at nns_path
  int fd = open(nns_path.c_str(), O_RDONLY);
  if(fd == -1) {
    std::cerr << "Failure opening NNS " << nns_name
              << " at path " << nns_path << "\n"
              << strerror(errno) << "\n";
             
    assert(0);
  }

  // set the namespace
  int status = setns(fd, CLONE_NEWNET);
  if(status != 0) {
    std::cerr << "Failure setting NNS " << nns_name
              << " at path " << nns_path << "\n"
              << strerror(errno) << "\n";
    assert(0);
  }

  // done
  std::cout << "NNS " << nns_name << " set.\n";
}

