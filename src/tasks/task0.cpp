#include "tasks/task.hpp"
#include "controllers.hpp"
#include <iostream>
#include <signal.h>

/*
    task0 used to calibrate robot's parameters
*/

double height;

void task0Descent(Robot &r, SE3 &T_des){
    ros::Rate loop_rate(LOOP_FREQUENCY);
    VEC6 q0 = r.joints.q();
    VEC3 q_gripper = r.joints.q_gripper();
    VEC6 q_des;
    q_des = r.inverseKinematics(T_des);
    velocityController(r, DT, VELOCITY, q_des, false);
    r.joints.update();
}

void signal_callback_handler(int signum) {
   std::cout << "Height: " << height << std::endl;
   // Terminate program
   exit(signum);
}

bool task0(ros::ServiceClient &detectClient){
    signal(SIGINT, signal_callback_handler);
    VEC3 STATION;
    VEC3 pose;
    height = 0.60;
    pose << 0.30, -0.24, height;
    SE3 T_des;
    T_des = SE3Operations::getGripperPose(pose, 0);
    ur5.move(T_des);
    ROS_INFO_STREAM("Destination reached");

    while(ros::ok()){
        height += 0.005;
        T_des(2, 3) = height;
        task0Descent(ur5, T_des);
        ros::Duration(0.5).sleep();
    }
    
    return true;
}