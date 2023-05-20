#ifndef FILE_MANAGER_HPP_
#define FILE_MANAGER_HPP_

/*
文件读写管理程序。文件的内容以流的方式存储。构造函数中打开文件，析构函数关闭文件，防止文件没打开就使用或被错误关闭。
*/

#include <string>
#include <iostream>
#include <fstream>
#include "glog/logging.h"

enum class FileMode
{
  write,
  read,
  append
};

class FileManager
{
 public:
  std::string directoryPath;
  std::string filename;
  std::string filePath;
  std::fstream fileStream;
  FileMode filemode;
  FileManager(std::string directoryPathIn, std::string nameIn, FileMode modeIn) :
    directoryPath(directoryPathIn),
    filename(nameIn),
    filePath(directoryPath + "/" + filename),
    filemode(modeIn)
  {
    initializeDirectory( );
  }

  // 默认创建的文件夹和文件
  FileManager( ) :
    directoryPath(std::getenv("HOME") + std::string("/temp")),
    filename("temp.txt"),
    filePath(directoryPath + "/" + filename)
  {
    initializeDirectory( );
  }

  // 复制构造函数
  FileManager(const FileManager &copy_from) :
    directoryPath(copy_from.directoryPath),
    filename(copy_from.filename),
    filePath(copy_from.filePath)
  {
    initializeDirectory( );
  }

  void initializeDirectory( )
  {
    // 递归地删除该文件夹； c_str() 函数返回一个指向正规C字符串的指针常量,内容与本 string 串相同；
    //  int unused = system((std::string("exec rm -r ") + directoryPath).c_str()); // system正常执行返回 0
    //  在父文件夹中创建文件夹，若父文件夹不存在则先创建父文件夹
    int unused = system((std::string("mkdir -p ") + directoryPath).c_str( ));
    switch (filemode)
    {
      case FileMode::write: {
        fileStream.open(filePath, std::ios::out);
        break;
      }
      case FileMode::read: {
        fileStream.open(filePath, std::ios::in);
        break;
      }
      case FileMode::append: {
        fileStream.open(filePath, std::ios::app);
        break;
      }
    }
  }

  template <typename T>
  std::ofstream &operator<<(T data)
  {
    fileStream << data;
  }

  ~FileManager( )
  {
    fileStream.close( );
  }
};

#endif

