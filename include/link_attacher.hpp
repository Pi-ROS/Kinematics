#ifndef LINK_ATTACHER_HPP
#define LINK_ATTACHER_HPP

#include <ros/ros.h>
#include "gazebo_ros_link_attacher/Attach.h"

static const char ur5_model[] = "ur5";
static const char ur5_hand_link[] = "hand_1_link";
static const char table_model[] = "tavolo";
static const char table_link[] = "link";

extern ros::ServiceClient attach_client;
extern ros::ServiceClient detach_client;
void attach(const char* model1, const char* link1, const char* model2, const char* link2);
void detach(const char* model1, const char* link1, const char* model2, const char* link2);

#endif