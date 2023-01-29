#include "ros/ros.h"
#include "../include/kinematics.hpp"
#include <math.h>
#include <iostream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle node;

    pub_des_jstate = node.advertise<sensor_msgs::JointState>("/command", 1);

    ros::Rate loop_rate(loop_frequency);

    jointState_msg_sim.position.resize(6);
    jointState_msg_sim.velocity.resize(6);
    jointState_msg_sim.effort.resize(6);
    
    q_des0 << -0.3223527113543909, -0.7805794638446351, -2.5675506591796875, -1.6347843609251917, -1.5715253988849085, -1.0017417112933558;

    while (ros::ok())
    {
        std::cout << "Hello, world!" << std::endl;
    }

    return 0;
}
