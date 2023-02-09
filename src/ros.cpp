#include "ros.hpp"
#include "../include/config.hpp"

void publishJoints(ros::Publisher &pub, JointStateVector &data) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(data.size());
    for (int i = 0; i < data.size(); i++)
    {
        msg.data[i] = data(i,0);
    }
    pub.publish(msg);
}

/**
 * @brief Different implementations of the publisher depending on the gripper type
*/

#if SOFT_GRIPPER
void publishJointsAndGripper(ros::Publisher &pub, JointStateVector &qJ, VEC3 &qG) {
    std_msgs::Float64MultiArray msg;
    // 6 joints + 2 gripper
    msg.data.resize(qJ.size() + qG.size() - 1);
    for (int i = 0; i < qJ.size(); i++)
    {
        msg.data[i] = qJ(i,0);
    }
    msg.data[6] = qG(0,0);
    msg.data[7] = qG(1,0);
    pub.publish(msg);
}
#else
void publishJointsAndGripper(ros::Publisher &pub, JointStateVector &qJ, VEC3 &qG) {
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

void readJoints(const sensor_msgs::JointState::ConstPtr& msg) {
    for (int i = 0; i < data_read.size(); i++)
    {
        //ROS_INFO_STREAM("INSIDE position " << i << " : " << msg->position[i]);
        data_read(i,0) = msg->position[i];
    }
}

// debug - float64multiArray message
void readJointsDebug(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    for (int i = 0; i < data_read.size(); i++)
    {
        //ROS_INFO_STREAM("data " << i << " : " << msg->data[i]);
        data_read(i,0) = msg->data[i];
    }
}

