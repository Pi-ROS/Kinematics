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
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    
    /* robot state initialization */
    ur5 = Robot(q_home);
    ur5.joints.update();
    ur5.moveGripper(gripperClient, 180, 10, 0.1);

    #if TASK0
    /* robot calibration task */
    ur5.moveGripper(180, 10, 0.1);
    task0(detectClient);
    ros::spin();
    #else

    // FIXME remove comments - debug only

    // /* gripper service */
    // gripperClient = node.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
    // gripperClient.waitForExistence();
    // /* zed camera service */
    // detectClient = node.serviceClient<pijoint_vision::ObjectDetection>("object_detection");
    // detectClient.waitForExistence();
    
    /* task selection */
    switch (TASK_SELECTION)
    {
        case 1:
            task1(detectClient, gripperClient);
            break;
        case 2:
            task2(detectClient, gripperClient);
            break;
        case 3:
            task3(detectClient, gripperClient);
            break;
        case 4:
            task4(detectClient, gripperClient);
            break;
        case 5:
            task01(detectClient, gripperClient);
            break;
        default:
            ROS_INFO_STREAM("Invalid task selection");
            exit(1);
            break;
    }

    ros::spin();
    #endif
}
