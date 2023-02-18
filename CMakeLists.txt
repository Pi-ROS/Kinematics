
cmake_minimum_required(VERSION 2.8.3)
project(kinematics)
 
## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs pijoint_vision sensor_msgs message_generation)
find_package(Eigen3 3.3 REQUIRED)

## Declare a catkin package
add_service_files(FILES
  Task.srv
)
generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   
)
catkin_package()
 
## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(kinematics_node src/kinematics.cpp src/robot.cpp src/ros.cpp src/controllers.cpp 
src/utils.cpp src/tasks/task0.cpp src/tasks/task1.cpp src/tasks/task2.cpp src/tasks/task3.cpp
src/tasks/task4.cpp src/tasks/task01.cpp src/link_attacher.cpp)


target_link_libraries(kinematics_node ${catkin_LIBRARIES})
install(TARGETS kinematics_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
