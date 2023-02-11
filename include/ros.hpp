#ifndef ROS_HPP
#define ROS_HPP

#include "ros/ros.h"
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <pijoint_vision/ObjectDetection.h>

#include "se3.hpp"

#define LOOP_FREQUENCY 1. // [Hz]

static std::string debug_topic = "/debug";
static std::string joint_state_publisher_topic = "/ur5/joint_group_pos_controller/command";
static std::string joint_state_subscriber_topic = "/ur5/joint_states";

extern ros::Publisher pub_jstate;

/**
 * @brief Publishes the joints and gripper configurations
 * @param pub Publisher
 * @param qJ Joints configuration
 * @param qG Gripper configuration
*/
void publishJoints(ros::Publisher &pub, VEC6 &qJ, VEC3 &qG);

/**
 * @brief Reads the current configurations of the joints
 * @param msg Memory location where the message will be copied
*/
VEC9 readJoints();



void normalize(VEC6 &q);

#endif