cmake_minimum_required(VERSION 3.10)
project(callback)
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)

# ros pkg dep
set(ROS_DEP roscpp sensor_msgs)
# when create new message, build with message_generation
# find_package(catkin REQUIRED COMPONENTS message_generation ${ROS_DEP})
find_package(catkin REQUIRED COMPONENTS ${ROS_DEP})

catkin_package(CATKIN_DEPENDS ${ROS_DEP})

add_executable(${PROJECT_NAME}_node src/callback.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
target_include_directories(${PROJECT_NAME}_node ${catkin_INCLUDE_DIRS})



