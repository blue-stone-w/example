cmake_minimum_required(VERSION 3.10)
project(to_rosbag)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

# ros pkg dep
set(ROS_DEP roscpp sensor_msgs rosbag)
# when create new message, build with message_generation
# find_package(catkin REQUIRED COMPONENTS message_generation ${ROS_DEP})
find_package(catkin REQUIRED COMPONENTS ${ROS_DEP})

# catkin_package() 是catkin提供的CMake的宏，这个指令是用来提供一些catkin的信息来创建pkg-config和CMake文件。
# 这个指令必须在 add_library() 或 add_executable()前执行， 这个指令有 5 个可选参数：
# INCLUDE_DIRS - 被这个package导出的include路径 
# LIBRARIES - 被这个package导出的library
# CATKIN_DEPENDS - 这个工程依赖的其他catkin projects 
# DEPENDS - 这个工程依赖的Non-catkin CMake projects 
# CFG_EXTRAS - 其他的构建选项。
# catkin_package(CATKIN_DEPENDS ${ROS_DEP} message_runtime)
catkin_package(CATKIN_DEPENDS ${ROS_DEP})


# file() 
aux_source_directory(convert_to_rosbag ALL_SRC)
add_executable(${PROJECT_NAME} src/readbag.cpp)
# add_executable(${PROJECT_NAME} ${ALL_SRC})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME} PUBLIC ./convert_to_rosbag ${catkin_INCLUDE_DIRS})

# 
set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./)

