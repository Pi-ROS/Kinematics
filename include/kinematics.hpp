#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> JointStateVector;

JointStateVector inverseKinematicsSolver(/* TODO: params */);
void send_des_jstate(const JointStateVector &joint_pos);

JointStateVector q_des = JointStateVector::Zero();
JointStateVector q_des0 = JointStateVector::Zero();
JointStateVector qd_des = JointStateVector::Zero();
JointStateVector tau_ffwd = JointStateVector::Zero();
JointStateVector filter_1 = JointStateVector::Zero();
JointStateVector filter_2 = JointStateVector::Zero();

double loop_time = 0.;
double loop_frequency = 1000.;

ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;

#endif