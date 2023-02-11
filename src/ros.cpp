#include "ros.hpp"
#include "../include/config.hpp"


void normalize(VEC6 &q){

    for (int i = 0; i < q.size(); i++)
    {

        double value = q(i);
        if(value > 2*M_PI) value -=2*M_PI;
        if(value < -2*M_PI) value +=2*M_PI;
        q(i) = value;
    }
}

/**
 * @brief Different implementations of the publisher depending on the gripper type
*/
#if !(USE_GRIPPER)
void publishJoints(ros::Publisher &pub, VEC6 &qJ, VEC3 &qG) {
    std_msgs::Float64MultiArray msg;
    // 6 joints + 2 gripper
    msg.data.resize(qJ.size() + qG.size() - 1);
    for (int i = 0; i < qJ.size(); i++)
    {
        msg.data[i] = qJ(i,0);
    }
    pub.publish(msg);
}
#elif SOFT_GRIPPER
void publishJoints(ros::Publisher &pub, VEC6 &qJ, VEC3 &qG) {
    std_msgs::Float64MultiArray msg;
    // 6 joints + 2 gripper
    msg.data.resize(qJ.size() + qG.size() - 1);
    for (int i = 0; i < qJ.size(); i++)
    {
        msg.data[i] = qJ(i);
    }

    msg.data[6] = qG(0,0);
    msg.data[7] = qG(1,0);

    pub.publish(msg);
}
#else
void publishJoints(ros::Publisher &pub, VEC6 &qJ, VEC3 &qG) {
    std_msgs::Float64MultiArray msg;
    // 6 joints + 3 gripper
    msg.data.resize(qJ.size() + qG.size());
    for (int i = 0; i < qJ.size(); i++)
    {
        msg.data[i] = qJ(i,0);
    }
    msg.data[6] = qG(0,0);
    msg.data[7] = qG(1,0);
    msg.data[8] = qG(2,0);
    pub.publish(msg);
}
#endif

/*
- elbow_joint
- hand_1_joint
- hand_2_joint
  - hand_3_joint
  - shoulder_lift_joint
  - shoulder_pan_joint
  - wrist_1_joint
  - wrist_2_joint
  - wrist_3_joint

*/

#if !(USE_GRIPPER)
VEC9 readJoints() {
    sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_subscriber_topic);
    VEC9 data_read;
    data_read.resize(9);

    data_read(2,0) = msg->position[0];
    data_read(1,0) = msg->position[1];
    data_read(0,0) = msg->position[2];
    for (int i = 3; i < data_read.size(); i++)
    {
        //ROS_INFO_STREAM("INSIDE position " << i << " : " << msg->position[i]);
        data_read(i,0) = msg->position[i];
    }
    return data_read;
}
#elif SOFT_GRIPPER
VEC9 readJoints() {
    sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_subscriber_topic);
    VEC9 data_read;
    data_read.resize(9);

    data_read(2) = msg->position[0];
    data_read(6) = msg->position[1];
    data_read(7) = msg->position[2];
    data_read(1) = msg->position[3];
    data_read(0) = msg->position[4];
    data_read(3) = msg->position[5];
    data_read(4) = msg->position[6];
    data_read(5) = msg->position[7];
    return data_read;
}
#else
VEC9 readJoints() {
    sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>(joint_state_subscriber_topic);
    VEC9 data_read;
    data_read.resize(9);

    data_read(2) = msg->position[0];
    data_read(6) = msg->position[1];
    data_read(7) = msg->position[2];
    data_read(1) = msg->position[3];
    data_read(0) = msg->position[4];
    data_read(3) = msg->position[5];
    data_read(4) = msg->position[6];
    data_read(5) = msg->position[7];
    return data_read;
}
#endif