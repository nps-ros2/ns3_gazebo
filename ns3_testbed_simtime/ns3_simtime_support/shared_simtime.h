#ifndef SHARED_SIMTIME_H
#define SHARED_SIMTIME_H

class shared_simtime_t {
  private:
  int fd;
  uint64_t* time_ptr;

  public:
  shared_simtime_t();
  void set_t(uint64_t t);
  uint64_t t() const;
};

#endif

