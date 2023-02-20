#ifndef TASK_HPP
#define TASK_HPP

#include "config.hpp"
#include "robot.hpp"
#include "ros.hpp"
#include "utils.hpp"
#include <kinematics/TaskResponse.h>

#define ARRAY_LENGTH(array) (sizeof(array) / sizeof(array[0]))

extern Robot ur5;

/**
 * @brief Mapping between class indices and class names.
 */
static const std::string targetNames[11] = {
    "X1-Y1-Z2","X1-Y2-Z1", "X1-Y2-Z2", "X1-Y2-Z2-CHAMFER",
    "X1-Y2-Z2-TWINFILLET", "X1-Y3-Z2", "X1-Y3-Z2-FILLET",
    "X1-Y4-Z1", "X1-Y4-Z2", "X2-Y2-Z2", "X2-Y2-Z2-FILLET"
};

/**
 * @brief Simple task to test the robot.
 */
bool task0(ros::ServiceClient &detectClient);

/**
 * @brief Another simple task to test the robot.
 */
bool task01(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient);

/**
 * @brief First task: pick up one brick and move it to some desired
 * position.
 */
bool task1(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient);

/**
 * @brief Second task: pick up some number of bricks and move them to
 * the position designated by their class.
 */
bool task2(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient);

/**
 * @brief Third task: pick up some number of bricks and move them to
 * the position designated by their class. More than one brick for each
 * class may be present on the table.
 */
bool task3(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient);

/**
 * @brief Final task: build a small castle.
 */
bool task4(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient);

bool task5(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient);

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
static double classHeight[11] = {0.06, 0.035, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.065, 0.06, 0.06};

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

int getModelFromPosition(double x, double y);
void task3v1Descent(Robot &ur5, SE3 T_des, bool pick, int brick_class, char* detected_model);

static const char* brick_models_task4[] = {
    "x1-y4-z2", 
    "x1-y4-z2-2",
    "x1-y4-z1",
    "x1-y4-z1-2",
};

static const char* brick_models_task3[] = {
    "x1-y4-z2", 
    "x1-y4-z2-2",
    "x1-y4-z2-3",
    "x1-y2-z2",
};

static const double models_positions[][2] = {
    {-0.447, -0.303},
    {-0.253, -0.303},
    {-0.053, -0.303},
    {0.147, -0.303},
};

#endif