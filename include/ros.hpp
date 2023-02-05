#ifndef ROS_HPP
#define ROS_HPP

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include "se3.hpp"

#define LOOP_FREQUENCY 10. // [Hz]

extern JointStateVector data_read; // defined in kinematics.hpp

void publishJoints(ros::Publisher pub, JointStateVector &data);
void readJoints(const sensor_msgs::JointState::ConstPtr& msg);
void readJointsDebug(const std_msgs::Float64MultiArray::ConstPtr& msg);

#endif