#ifndef TASK_HPP
#define TASK_HPP

#include "config.hpp"
#include "robot.hpp"
#include "ros.hpp"
#include "se3.hpp"
#include <kinematics/Task.h>

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))

extern Robot ur5;

static const std::string targetNames[11] = {
    "X1-Y1-Z2","X1-Y2-Z1", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2", "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2", "X2-Y2-Z2-FILLET"
};
bool task1(ros::ServiceClient &detect);
bool task2(ros::ServiceClient &detect);
bool task3(ros::ServiceClient &detect);
bool task4(ros::ServiceClient &detect);

static int nextAvailableTargetPosition = 0;
static VEC3 targetPositions[] = {
    VEC3(0.40, 0, DESCENT_HEIGHT),
    VEC3(0.40, -0.15, DESCENT_HEIGHT),
    VEC3(0.40, -0.3, DESCENT_HEIGHT),
    VEC3(0.25, 0, DESCENT_HEIGHT),
    VEC3(0.25, -0.15, DESCENT_HEIGHT),
    VEC3(0.25, -0.3, DESCENT_HEIGHT),
};
static int classTargetPositions[11] = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

inline int getClassTargetPosition(int classId) {
    if (classTargetPositions[classId] == -1) {
        if (nextAvailableTargetPosition >= ARRAY_LENGTH(targetPositions)) {
            ROS_INFO_STREAM("CANNOT HAVE MORE THAN " << ARRAY_LENGTH(targetPositions) << " BRICK CLASSES AT THE SAME TIME");
            exit(0);
        }
        classTargetPositions[classId] = nextAvailableTargetPosition;
        nextAvailableTargetPosition++;
    }
    return classTargetPositions[classId];
}

#endif