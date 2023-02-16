#include "tasks/task.hpp"

bool task01(ros::ServiceClient &detect)
{
    VEC3 destination;
    VEC3 pose;
    destination << 0.24, -0.24, DESCENT_HEIGHT;
    pose << -0.24, -0.24, WORKING_HEIGHT;

    double block_rotation = 0;
    int class_id = 8;

    // Reach the brick
    SE3 T_des;
    T_des = SE3Operations::getGripperPose(pose, block_rotation);
    ur5.move(T_des);
    T_des(2, 3) = DESCENT_HEIGHT;
    ur5.descent(T_des, true);

    // Move to the final position
    pose << destination(0), destination(1), WORKING_HEIGHT;
    T_des = SE3Operations::getGripperPose(pose, M_PI / 2);
    ur5.move(T_des);
    T_des(2, 3) = destination(2);
    ur5.descent(T_des, false);

    return true;
}