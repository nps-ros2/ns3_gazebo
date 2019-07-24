#ifndef PIPE_WRITER_HPP
#define PIPE_WRITER_HPP

#include <string>

class pipe_writer_t {

  private:
  const bool no_pipe;
  const int fd;

  public:
  pipe_writer_t(bool _no_pipe);
  void log(std::string text);
};

#endif

