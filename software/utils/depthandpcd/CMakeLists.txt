cmake_minimum_required(VERSION 3.6)
project(depthandpcd)

cmake_policy(SET CMP0148 OLD)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  message_filters
  sensor_msgs
)

find_package(OpenCV REQUIRED)
find_package(Python REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
	
)

add_executable(depthandpcd_node src/test.cpp)
target_link_libraries(depthandpcd_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})