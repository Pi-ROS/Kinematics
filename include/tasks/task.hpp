#ifndef TASK_HPP
#define TASK_HPP

#include "config.hpp"
#include "robot.hpp"
#include "ros.hpp"
#include "se3.hpp"
#include <kinematics/Task.h>

extern Robot ur5;

static const std::string targetNames[11] = {
    "X1-Y1-Z2","X1-Y2-Z1", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2", "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2", "X2-Y2-Z2-FILLET"
};

static VEC3 targetPositions[11] = {
    VEC3(0.40, 0.15, 0.72),
    VEC3(0.40, 0, 0.72),
    VEC3(0.40, -0.15, 0.72),
    VEC3(0.40, -0.30, 0.72),
    VEC3(-1, -1, 1),  // This class is for the x1-y2-z2-twinfillet block
    VEC3(0.3, -0.3, 0.72),
    VEC3(0.3, 0, 0.72),
    VEC3(0.3, -0.15, 0.72),
    VEC3(0.3, -0.3, 0.72),
    VEC3(0.2, -0.15, 0.72),
    VEC3(0.2, -0.3, 0.72)
};

bool task1(ros::ServiceClient &detect);
bool task2(ros::ServiceClient &detect);
bool task3(ros::ServiceClient &detect);
bool task4(ros::ServiceClient &detect);

#endif