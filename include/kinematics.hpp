#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "ros.hpp"
#include "robot.hpp"
#include <string>

static bool DEBUG = false;


VEC9 data_read;
ros::Publisher pub_jstate;
ros::Subscriber sub_jstate;

#endif