#ifndef PIPE_WRITER_HPP
#define PIPE_WRITER_HPP

#include <string>

class pipe_writer_t {

  private:
  const bool use_pipe;
  const int fd;

  public:
  pipe_writer_t(bool _use_pipe);
  void log(std::string text);
};

#endif

