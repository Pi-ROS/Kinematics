#include "kinematics.hpp"
#include "tasks/task.hpp"
#include "ros.hpp"

int main(int argc, char **argv)
{
    ROS_INFO_STREAM("STARTING KINEMATICS NODE");
    ros::init(argc, argv, "kinematics");
    ros::NodeHandle node; 
    pub_jstate = node.advertise<std_msgs::Float64MultiArray>(joint_state_publisher_topic, 10);
    VEC6 q_home;


    #if TASK0
    /* robot configuration */
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    ur5 = Robot(q_home);
    ur5.moveGripper(180, 10, 0.1);
    task0(detect);
    ros::spin();
    #endif

    /* zed camera service */
    detect = node.serviceClient<pijoint_vision::ObjectDetection>("object_detection");
    detect.waitForExistence();

    /* gripper service */
    #if !(SIMULATION)
    gripperClient = node.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
    gripperClient.waitForExistence();
    #endif

    
    /* robot state initialization */
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    ur5 = Robot(q_home);
    ur5.moveGripper(180, 10, 0.1);

    
    task0(detect);
    ros::spin();
}
