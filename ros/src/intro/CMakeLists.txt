# 指定cmake的最低版本。在实际使用时，如果涉及的项目不复杂，随便选择一个版本即可。如果报错了再重选最低版本即可。
cmake_minimum_required(VERSION 3.10)
# 指定该项目的名称，此处该项目的名称即为“intro”
project(intro)
# 编译类型为release，不是debug
set(CMAKE_BUILD_TYPE Debug)
# 设置C++编译选项，使用C++14版本标准库；
set(CMAKE_CXX_STANDARD 14)

# ros pkg dep
set(ROS_DEP roscpp sensor_msgs)
# 指定编译时需要依赖的其它包或者库
find_package(catkin REQUIRED COMPONENTS ${ROS_DEP})
find_package(OpenCV REQUIRED QUIET)

# 添加消息文件以生成自定义消息。
add_message_files(
  DIRECTORY msg # 自定义的消息文件所在的文件夹
  FILES cloud_info.msg # 自定义的消息文件，此处自定义了名为“cloud_info.msg”的文件
)
# 添加服务文件以生成自定义服务。
add_service_files(
  DIRECTORY srv # 自定义的服务文件所在的文件夹
  FILES save_map.srv # 自定义的服务文件，此处自定义了名为“save_map.srv”的文件
)

# TODO 动作action

# 生成消息、服务或动作；指定生成时需要的依赖项
generate_messages(DEPENDENCIES
  std_msgs
  sensor_msgs
)

# 指定catkin信息给编译器以生成cmake文件
catkin_package(
  INCLUDE_DIRS include # 指定包含头文件的文件夹
  CATKIN_DEPENDS ${ROS_DEP})

# 把文件生成为可执行文件。括号中为参数。第一个参数为生成的可执行文件的名称，其余参数为需要用到的文件。此处仅写了两个文件，可以继续添加
add_executable(${PROJECT_NAME}_node src/main.cpp src/2.cpp) 
# 为可执行文件链接库。括号中为参数。第一个参数为可执行文件的名称，其余参数为库的路径。此处添加了两个路径参数
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
# 为可执行文件添加头文件包含路径。第一个参数为可执行文件的名称，其余参数为头文件的路径。此处添加了一个路径参数
target_include_directories(${PROJECT_NAME}_node ${catkin_INCLUDE_DIRS})
# 把文件生成为库文件
# add_libtrary(${PROJECT_NAME}_lib src/callback.cpp) 


