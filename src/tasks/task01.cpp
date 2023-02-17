#include "tasks/task.hpp"

bool task01(ros::ServiceClient &detectClient, ros::ServiceClient &gripperClient)
{
    VEC3 destination;
    VEC3 pose;
    destination << 0.24, 0, WORKING_HEIGHT;
    pose << 0.24, -0.24, WORKING_HEIGHT;

    double block_rotation = 0;
    int class_id = 8;

    // Reach the brick
    SE3 T_des;
    T_des = SE3Operations::getGripperPose(pose, block_rotation);
    ur5.move(T_des);
    T_des(2, 3) = DESCENT_HEIGHT;
    ur5.descent(T_des, true, gripperClient);

    // Move to the final position
    T_des = SE3Operations::getGripperPose(destination, M_PI / 2);
    ur5.move(T_des);
    T_des(2, 3) = DESCENT_HEIGHT;
    ur5.descent(T_des, false, gripperClient);

    return true;
}