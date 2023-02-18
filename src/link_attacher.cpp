#include "link_attacher.hpp"

ros::ServiceClient attach_client;
ros::ServiceClient detach_client;
gazebo_ros_link_attacher::Attach link_attacher_srv;

void attach(const char* model1, const char* link1, const char* model2, const char* link2)
{
    link_attacher_srv.request.model_name_1 = model1;
    link_attacher_srv.request.link_name_1 = link1;
    link_attacher_srv.request.model_name_2 = model2;
    link_attacher_srv.request.link_name_2 = link2;
    attach_client.call(link_attacher_srv);
}

void detach(const char* model1, const char* link1, const char* model2, const char* link2)
{
    link_attacher_srv.request.model_name_1 = model1;
    link_attacher_srv.request.link_name_1 = link1;
    link_attacher_srv.request.model_name_2 = model2;
    link_attacher_srv.request.link_name_2 = link2;
    detach_client.call(link_attacher_srv);
}