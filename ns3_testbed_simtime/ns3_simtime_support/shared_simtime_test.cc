#include <stdint.h> // uint64_t
#include <iostream> // std::cout
#include <cassert>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h> // flags
#include "shared_simtime.h"

int main(int argc, char *argv[]) {
  shared_simtime_t shared_time;
  shared_time.set_t(2L);
  uint64_t t = shared_time.t();
  std::cout << "t2L: " << t << "\n";
  assert(t==2);

  shared_time.set_t(3L);
  t = shared_time.t();
  std::cout << "t23: " << t << "\n";
  assert(t==3);
}

