
cmake_minimum_required(VERSION 2.8.3)
project(kinematics)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(Eigen3 3.3 REQUIRED)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(kinematics_node src/kinematics.cpp src/robot.cpp src/ros.cpp src/se3.cpp)
target_link_libraries(kinematics_node ${catkin_LIBRARIES})
install(TARGETS kinematics_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
