#include "kinematics.hpp"


int main(int argc, char **argv)
{
    JointStateVector joint_pos;
    joint_pos.resize(6);
    data_read.resize(6);
    

    ros::init(argc, argv, "kinematics");
   
    ROS_INFO_STREAM("STARTING KINEMATICS NODE");
    if (argc == 2)
    {
        DEBUG = true;
        ROS_INFO_STREAM("--- DEBUG MODE ---");
    }

    ros::NodeHandle node;

    if (DEBUG)
    {
        pub_jstate = node.advertise<std_msgs::Float64MultiArray>(debug_topic, 10);
        //sub_jstate = node.subscribe(debug_topic, 1000, readJointsDebug);
    }
    else
    {
        pub_jstate = node.advertise<std_msgs::Float64MultiArray>(joint_state_publisher_topic, 10);
        //sub_jstate = node.subscribe(joint_state_subscriber_topic, 1000, readJoints);
    }

    /* zed camera service
    ros::ServiceClient client = node.serviceClient<pijoint_vision::ObjectDetection>("object_detection");
    pijoint_vision::ObjectDetection detection_srv;
    
    detection_srv.request.detect = true;
    client.waitForExistence();

    pijoint_vision::Object obj;
    VEC3 pose;

    if (client.call(detection_srv))
    {
        if(detection_srv.response.success)
        {
            ROS_INFO_STREAM("Object detected");
            obj = detection_srv.response.objects[0];
            pose << obj.box.center.x, obj.box.center.y, obj.box.center.z;
            ROS_INFO_STREAM("Pose:\n" << pose);
        }
        else
        {
            ROS_INFO_STREAM("Object not detected");
        }
    }

    */

    ros::Rate loop_rate(LOOP_FREQUENCY);
    ros::spinOnce();
    loop_rate.sleep();


    VEC6 q_home;
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    Robot ur5(q_home);
    
    VEC3 pose;
    pose << -0.24, -0.24, 0.60;
    ur5.move(pose);
    ur5.descent(0.6);
    // TODO sleed in maniera piú furba
    //ur5.descend(0.6);

    pose << 0.24, -0.24, 0.60;
    ur5.move(pose);
    ur5.descent(0.6);

    pose << 0.24, -0.12, 0.60;
    ur5.move(pose);
    ur5.descent(0.6);

    pose << 0.24, -0.24, 0.60;
    ur5.move(pose);
    ur5.descent(0.6);

    pose << -0.24, -0.24, 0.60;
    ur5.move(pose);
    ur5.descent(0.6);

    while (ros::ok())
    {        
        loop_rate.sleep();   
    }

    return 0;
}
