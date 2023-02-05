#include "ros.hpp"

void publishJoints(ros::Publisher pub, JointStateVector &data) {
    std_msgs::Float64MultiArray msg;
    msg.data.resize(data.size());
    for (int i = 0; i < data.size(); i++)
    {
        msg.data[i] = data(i,0);
    }
    pub.publish(msg);
}

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

