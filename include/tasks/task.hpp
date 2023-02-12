#ifndef TASK_HPP
#define TASK_HPP

#include "config.hpp"
#include "robot.hpp"
#include "ros.hpp"
#include "se3.hpp"
#include <kinematics/Task.h>

extern Robot ur5;
static VEC3 targetPositions[11] = {
    VEC3(0.40, 0, 0.72),
    VEC3(0.40, -0.1, 0.72),
    VEC3(0.40, -0.2, 0.72),
    VEC3(0.40, -0.3, 0.72),
    VEC3(-1, -1, 1),  // This class is for the x1-y2-z2-twinfillet block
    VEC3(0.3, -0.1, 0.72),
    VEC3(0.3, -0.2, 0.72),
    VEC3(0.3, -0.3, 0.72),
    VEC3(0.3, -0.4, 0.72),
    VEC3(0.2, 0, 0.72),
    VEC3(0.2, -0.1, 0.72)
};

bool task1(ros::ServiceClient &detect);
bool task2(ros::ServiceClient &detect);
bool task3(ros::ServiceClient &detect);
bool task4(ros::ServiceClient &detect);

#endif