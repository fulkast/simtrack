cmake_minimum_required(VERSION 2.8.7)
project(tracking_data_generator)

SET(CMAKE_CXX_COMPILER /usr/bin/g++-4.9)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  low_level_vision
  pose_estimation
  roscpp
)

find_package(OpenCV 3.0.0 REQUIRED)
find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS low_level_vision pose_estimation
  DEPENDS cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV2_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIR}
)
