
#ifndef TASK_HPP
#define TASK_HPP

#include "config.hpp"
#include "robot.hpp"
#include "ros.hpp"
#include "se3.hpp"
#include <kinematics/Task.h>

extern Robot ur5;
bool task1(ros::ServiceClient &detect);


#endif