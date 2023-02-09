#include "kinematics.hpp"

int main(int argc, char **argv)
{
    JointStateVector joint_pos;

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

    // zed camera service
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


    ros::Rate loop_rate(LOOP_FREQUENCY);
    ros::spinOnce();
    loop_rate.sleep();

    VEC6 q0;
    q0 << 5.5127358094431145e-05, -0.0001212409520094937, 9.891741610346116e-05, 0.00013209079602738427, 4.505457065562268e-05, 6.162511098306567e-05;
    Robot ur5(q0);

    joint_pos.resize(6);
    data_read.resize(6);

    VEC6 q_home;
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    ur5.q = q_home;

    ROS_INFO_STREAM("STARTING spostamento piano");

    //  -0.1200 0.4243 (-0.1131)

    VEC3 tau_des;
    tau_des << pose(0), pose(1), Robot::workingHeight;
    Controller::redundantController(ur5, tau_des);
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO_STREAM("HO FINITO spostamento piano");
    VEC6 back = ur5.q;
    ros::Duration(0.5).sleep();

    ROS_INFO_STREAM("STARTING discensa");
    SE3 T_des;
    T_des <<    1, 0, 0, pose(0,0),
                0, 1, 0, pose(1,0),
                0, 0, 1, pose(2,0),
                0, 0, 0, 1;
    VEC6 q_des;
    q_des = ur5.inverseKinematics(T_des);
    ur5.q = q_des;
    publishJoints(pub_jstate, q_des);
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM("HO FINITO discesa");
    ros::Duration(0.5).sleep();

    ur5.moveGripper(30, 10, 0.1);
    ROS_INFO_STREAM("HO FINITO chiusura pinza");
    ros::Duration(0.5).sleep();

    ROS_INFO_STREAM("STARTING salita");
    publishJoints(pub_jstate, back);
    ros::spinOnce();
    loop_rate.sleep();
    ROS_INFO_STREAM("HO FINITO salita");


    while (ros::ok())
    {        
        loop_rate.sleep();   
    }

    return 0;
}
