
#ifndef TIME_HEADER_H_
#define TIME_HEADER_H_

#include "lib3rd/cpp_header.h"

class TicToc
{
 public:
  TicToc()
  {
    tic();
  }

  void tic()
  {
    start = std::chrono::system_clock::now();
  }

  double toc()
  {
    end                                           = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end - start;
    return elapsed_seconds.count() * 1000;
  }

 private:
  std::chrono::time_point<std::chrono::system_clock> start, end;
};

class Timer
{
 public:
  Timer()
  {
    restart();
  }

  // 重置计时器 reset timer
  void restart()
  {
    start_time = std::chrono::steady_clock::now();
  }

  // 结束计时,返回计时结果，单位为毫秒
  double elapsed(bool restart = false)
  {
    end_time                           = std::chrono::steady_clock::now();
    std::chrono::duration<double> diff = end_time - start_time;
    if (restart)
    {
      this->restart();
    }
    return diff.count() * 1000;
  }

 private:
  std::chrono::steady_clock::time_point start_time;
  std::chrono::steady_clock::time_point end_time;
};

#endif