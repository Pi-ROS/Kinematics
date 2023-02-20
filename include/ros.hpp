#ifndef ROS_HPP
#define ROS_HPP

#include "ros/ros.h"
#include "utils.hpp"

#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <pijoint_vision/ObjectDetection.h>
#include <ros_impedance_controller/generic_float.h>


#define LOOP_FREQUENCY 5 // [Hz]

static std::string joint_state_publisher_topic = "/ur5/joint_group_pos_controller/command";
static std::string joint_state_subscriber_topic = "/ur5/joint_states";

static ros::ServiceClient gripperClient;
static ros::ServiceClient detectClient;

extern ros::Publisher pub_jstate;

/**
 * @brief Publishes the joints and gripper configurations
 * @param pub Publisher
 * @param qJ Joints configuration
 * @param qG Gripper configuration
*/
void publishJoints(ros::Publisher &pub, VEC6 &qJ, VEC3 &qG);

/**
 * @brief Reads the current configurations of the joints and of the gripper.
 * @return The joint configuration.
*/
VEC9 readJoints();

/**
 * @brief Utility to ensure that the joint configuration 
 * is within the joint limits.
 * @param q The joint configuration.
 */
void normalize(VEC6 &q);

#endif