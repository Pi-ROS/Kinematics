#include "kinematics.hpp"
#include "tasks/task.hpp"
#include "link_attacher.hpp"
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

    #if TASK0
    /* robot calibration task */
    task0(detectClient);
    ros::spin();
    #else

    /* gripper service */
    gripperClient = node.serviceClient<ros_impedance_controller::generic_float>("move_gripper");
    gripperClient.waitForExistence();

    /* zed camera service */
    detectClient = node.serviceClient<pijoint_vision::ObjectDetection>("object_detection");
    detectClient.waitForExistence();

    #if SIMULATION && (TASK_SELECTION == 3 || TASK_SELECTION == 4 || TASK_SELECTION == 5)
    /* link attacher service */
    attach_client = node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/attach");
    attach_client.waitForExistence();
    detach_client = node.serviceClient<gazebo_ros_link_attacher::Attach>("link_attacher_node/detach");
    detach_client.waitForExistence();
    #endif

    /* gripper initialization */
    ur5.moveGripper(gripperClient, 180, 10, 0.1);
    
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
            task5(detectClient, gripperClient);
            break;
        case 6:
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
