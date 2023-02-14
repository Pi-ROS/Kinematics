#include "kinematics.hpp"
#include "tasks/task.hpp"
#include "ros.hpp"



int main(int argc, char **argv)
{
    VEC6 joint_pos;
    joint_pos.resize(6);

    ROS_INFO_STREAM("STARTING KINEMATICS NODE");
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle node; 
    pub_jstate = node.advertise<std_msgs::Float64MultiArray>(joint_state_publisher_topic, 10);

    /* zed camera service*/
    detect = node.serviceClient<pijoint_vision::ObjectDetection>("object_detection");
    detect.waitForExistence();

    #if !(SIMULATION)
    gripperClient = node.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
    gripperClient.waitForExistence();
    #endif

    
    // DEPLOY SERVICE

    VEC6 q_home;
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;

    ur5 = Robot(q_home);

    ur5.moveGripper(180, 10, 0.1);
    
    task4(detect);

    /*
    pijoint_vision::Object obj;
    VEC3 pose;
    double block_rotation;

    // while (1)
    // {
        pijoint_vision::ObjectDetection detection_srv;
        detection_srv.request.detect = true;
        if (client.call(detection_srv))
        {
            if (detection_srv.response.success && detection_srv.response.l > 0)
            {
                ROS_INFO_STREAM("Object detected");
                obj = detection_srv.response.objects[0];
                pose << obj.box.center.x, obj.box.center.y, obj.box.center.z;
                block_rotation = obj.box.rotation.yaw;
                ROS_INFO_STREAM("Pose:\n" << pose);
                // break;
            }
            else
            {
                ROS_INFO_STREAM("Object not detected");
                exit(0);
            }
        }
        else
        {
            ROS_INFO_STREAM("Failed to call service");
            exit(0);
        }
    // }

    ros::Rate loop_rate(LOOP_FREQUENCY);
    ros::spinOnce();
    loop_rate.sleep();

    
    ur5.moveGripper(100, 10, 0.01);

    //pose << -0.24, -0.24, 0.60;
    ur5.move(pose);
    ROS_INFO_STREAM("BLOCK ROATION" << block_rotation);
    ur5.descent(0.73, block_rotation, true);
    // TODO sleed in maniera piú furba
    // ur5.descend(0.6);


    ROS_INFO_STREAM("CI SPOSTIAMOOOOOOO");
    pose << 0.24, -0.24, 0.60;
    ur5.move(pose);
    ur5.descent(0.73, M_PI/2, false);

    while (ros::ok())
    {
        loop_rate.sleep();
    }

    return 0;*/
    ros::spin();
}
