#include <iostream>
#include <string>
#include <cstring>
#include <cassert>
#include <cstdlib> // getenv
#include <sys/types.h>
#include <sys/stat.h> // mkfifo
#include <sys/file.h> // lock
#include <errno.h>
#include <fcntl.h>
//#include <pwd.h>
#include <unistd.h>
#include <string.h> // strerror
#include "pipe_writer.hpp"

int _fd(bool use_pipe) {

  // no pipe
  if(!use_pipe) {
    return 0;
  }

  // filename
  std::string filename_string(getenv("HOME"));
  filename_string.append("/_testbed_pipe.pipe");

  const char *filename = filename_string.c_str();

  std::cerr << "PATH: '" << filename << "'\n";

  // maybe make fifo
  struct stat buffer;
  if(stat(filename, &buffer) != 0) {
    // no file so create pipe
    int status = mkfifo(filename, 0666);
    if(status != 0) {
      std::cerr << "Failure creating FIFO " << filename << "\n"
                << strerror(errno) << "\n";
      assert(0);
    }
  }

  // open fifo
  int fd = open(filename, O_WRONLY); // maybe not O_NONBLOCK
  if(fd == -1) {
    std::cerr << "Failure opening FIFO " << filename << "\n"
              << strerror(errno) << "\n";
    assert(0);
  }
  return fd;
}

pipe_writer_t::pipe_writer_t(bool _use_pipe) :
                    use_pipe(_use_pipe), fd(_fd(_use_pipe)) {
  std::cerr << "pipe_writer zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz" << use_pipe << "\n";
}

void pipe_writer_t::log(std::string text) {
  if(!use_pipe) {
    // no action
  }

  text.append("\n");
  size_t len = std::strlen(text.c_str());

  // lock
  int lock_status = flock(fd, LOCK_EX);
  if(lock_status != 0) {
    std::cerr << "Failure locking FIFO.\n"
              << strerror(errno) << "\n";
    assert(0);
  }

  // write
  ssize_t write_status = write(fd, text.c_str(), len);
  if (write_status < 0 || (size_t)write_status != len) {
    std::cerr << "failure writing FIFO: status: " << write_status
              << ", expected: " << len << "\n";
  }

  // unlock
  int unlock_status = flock(fd, LOCK_UN);
  if(unlock_status != 0) {
    std::cerr << "Failure unlocking FIFO.\n"
              << strerror(errno) << "\n";
    assert(0);
  }
} 

