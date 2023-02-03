#include "kinematics.hpp"

int main(int argc, char** argv){
    JointStateVector joint_pos ;

    ros::init(argc, argv, "direct_kinematics");

    ROS_INFO_STREAM("STARTING KINEMATICS NODE");
    if (argc == 2)
    {
        DEBUG = true;
        ROS_INFO_STREAM("--- DEBUG MODE ---");
    }

    ros::NodeHandle node;
    ros::Publisher pub_jstate;
    if (DEBUG) pub_jstate = node.advertise<std_msgs::Float64MultiArray>(debug_topic, 1);
    else pub_jstate = node.advertise<std_msgs::Float64MultiArray>(joint_state_topic, 1);

    ros::Rate loop_rate(LOOP_FREQUENCY);

    Robot ur5(JointStateVector::Zero());

    
    joint_pos.resize(6);
    joint_pos << 0.4, 0.3, 0.1, 0.3, 0.4, 0.2;

    while (ros::ok())
    {
        publishJoints(pub_jstate, joint_pos);
        SE3 ee = ur5.forwardKinematics(joint_pos);
        VEC3 ee_pos = SE3Operations::tra(ee);
        SO3 ee_rot = SE3Operations::rot(ee);
        ROS_INFO_STREAM("EE positions: ");
        ROS_INFO_STREAM(ee_pos(0) << " " << ee_pos(1) << " " << ee_pos(2));
        ROS_INFO_STREAM("EE rotation: ");
        ROS_INFO_STREAM(ee_rot(0) << " " << ee_rot(1) << " " << ee_rot(2));
        ROS_INFO_STREAM(ee_rot(3) << " " << ee_rot(4) << " " << ee_rot(5));
        ROS_INFO_STREAM(ee_rot(6) << " " << ee_rot(7) << " " << ee_rot(8));
        ROS_INFO_STREAM("--------------------");
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}