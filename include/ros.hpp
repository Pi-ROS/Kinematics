#ifndef ROS_HPP
#define ROS_HPP

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>
#include "se3.hpp"

#define LOOP_FREQUENCY 10. // [Hz]

void publishJoints(ros::Publisher pub, JointStateVector &data);

#endif