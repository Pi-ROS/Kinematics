#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "robot.hpp"
#include "ros.hpp"
#include "se3.hpp"
#include <string>

static bool DEBUG = false;


JointStateVector data_read;
ros::Publisher pub_jstate;
ros::Subscriber sub_jstate;

#endif