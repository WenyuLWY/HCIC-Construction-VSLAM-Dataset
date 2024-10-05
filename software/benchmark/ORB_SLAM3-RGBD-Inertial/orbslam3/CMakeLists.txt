cmake_minimum_required(VERSION 3.0.2)
project(orbslam3)

# set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_BUILD_TYPE "Debug")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy

)



include_directories(
)


# ${CERES_INCLUDE_DIRS}
catkin_package(
)

 

