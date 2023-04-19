cmake_minimum_required(VERSION 3.10.2) # VERSION 3.16.3
project(name)

# 编译类型为release，不是debug
set(CMAKE_BUILD_TYPE "Release")

# 设置C++编译选项，使用C++17版本标准库；
set(CMAKE_CXX_FLAGS "-std=c++17")

# 设置编译类型为Release时的编译选项：-03（最高级优化） -wall（开启警告） -g（开启调试）
# pthread是linux下的线程库，用了多线程就要链接这个库，这时候要在编译选项上增加-pthread或者-lpthread
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

include(cmake/glog.cmake)
include(cmake/PCL.cmake)
include(cmake/OpenCV.cmake)
include(cmake/YAML.cmake)
include(cmake/global_definition.cmake)
include(cmake/GTSAM.cmake)

#
# include_directories(libmy lib3rd)
include_directories(${PROJECT_SOURCE_DIR})

# ##########
# # Build ##
# ##########

add_executable(exe src/main.cpp)
target_link_libraries(exe ${PCL_LIBRARIES})

add_executable(space src/space.cpp)
target_link_libraries(space ${PCL_LIBRARIES})

# ---------------------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------------------
# install(TARGETS ${PROJECT_NAME}_node
# 	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# 	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# 	)

# install(DIRECTORY
#   launch
#   config
#   msg
#   srv
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#   )