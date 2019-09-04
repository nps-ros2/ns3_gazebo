#include <cassert>
#include <stdint.h> // uint64_t
#include <stdio.h>
#include <errno.h>

#include <iostream> // std::cout
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h> // flags
#include "shared_simtime.h"

shared_simtime_t::shared_simtime_t() : fd(0) {

  int open_flags = O_RDWR | O_CREAT;

  // shm_open
  fd = shm_open("/testbed_shared_simtime", open_flags, 0777);
  if(fd == -1) {
    perror("shm_open error");
    assert(0);
  }
  int status = ftruncate(fd, sizeof(uint64_t));
  if(status != 0) {
    perror("shared_simtime truncate");
    assert(0);
  }
  time_ptr = (uint64_t*)mmap(0, sizeof(uint64_t),
                        PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0);
  if (time_ptr == 0) {
    perror("sared_simtime pointer");
    assert(0);
  }
}

uint64_t shared_simtime_t::t() const {
  uint64_t t = *time_ptr;
  return t;
}

void shared_simtime_t::set_t(uint64_t t) {
  *time_ptr = t;
}

