#include "glog/logging.h"

#include "lib3rd/cpp_header.h"

/**** glog ****/
void(writer)(const char *data, int size)
{
  LOG(ERROR) << data;
}
int glog(char **argv)
{
  google::InitGoogleLogging(argv[0]);     // 初始化日志库
  FLAGS_minloglevel     = google::ERROR;  // 只记录大于等于ERROR等级的日志
  FLAGS_log_dir         = "./log/";       // 设置日志存放路径
  FLAGS_alsologtostderr = 1;              // 日志同时输出到stderr
  LOG(INFO) << "This is INFO data";       // 打印INFO等级日志
  LOG(WARNING) << "This is WARNING data"; // 打印WARNING等级日志
  LOG(ERROR) << "This is ERROR data";     // 打印ERROR等级日志
  LOG(FATAL) << "This is FATAL data";     // 打印FATAL等级日志
  {
    int i = 5;
    // log when meet condition
    LOG_IF(ERROR, i < 3) << i;
  }

  google::InstallFailureSignalHandler(); // 捕捉信号
  google::InstallFailureWriter(&writer); // 打印错误信息

  FLAGS_max_log_size              = 1;    // 单个日志文件大小上限（MB）, 如果设置为0将默认为1
  FLAGS_stop_logging_if_full_disk = true; // 当磁盘写满时，停止输出

  // 删除超过3天的所有文件，递归删除
  std::string rmfile = std::string("find ") + FLAGS_log_dir + " -mtime +3 -exec rm -rf {} \\;";
  if (0 > system(rmfile.data()))
  {
    printf("Exec cmd failed : %s\n", rmfile.data());
  }
  return 0;
}
