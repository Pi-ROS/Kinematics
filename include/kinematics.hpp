#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include "robot.hpp"
#include "ros.hpp"
#include <string>

static bool DEBUG = false;
static std::string debug_topic = "/debug";
static std::string joint_state_publisher_topic = "/ur5/joint_group_pos_controller/command";
static std::string joint_state_subscriber_topic = "/ur5/joint_states";


JointStateVector data_read;

#endif