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