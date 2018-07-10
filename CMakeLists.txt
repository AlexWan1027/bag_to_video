cmake_minimum_required(VERSION 2.8.3)
project(bag_to_video)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


add_executable(${PROJECT_NAME}_node src/bag_to_video_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}
)
