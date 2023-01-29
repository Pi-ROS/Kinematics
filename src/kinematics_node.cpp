#include "../include/kinematics.hpp"
#include <iostream>

const double loop_time = 0.;
const double loop_frequency = 1000.;

ros::Publisher pub_des_jstate;
sensor_msgs::JointState jointState_msg_sim;

void computeDesiredJointState();

int main(int argc, char **argv)
{
    ros::init(argc, argv, "kinematics_node");
    ros::NodeHandle node;

    // Load the robot model
    const char* locosim_dir = std::getenv("LOCOSIM_DIR");
    if (locosim_dir == NULL)
    {
        std::cout << "Please set the environment variable LOCOSIM_DIR to the root directory of the locosim repository." << std::endl;
        return 0;
    }
    std::string urdf_path = std::string(locosim_dir) + "/robot_urdf/ur5.urdf";
    pinocchio::Model robot = getRobotModel(urdf_path);

    // Set up the joint state publisher
    pub_des_jstate = node.advertise<sensor_msgs::JointState>("/command", 1);

    ros::Rate loop_rate(loop_frequency);

    jointState_msg_sim.position.resize(6);
    jointState_msg_sim.velocity.resize(6);
    jointState_msg_sim.effort.resize(6);    

    SE3 desired_pose;
    // The intial joint configuration
    // TODO: load the initial joint configuration
    JointStateVector q = JointStateVector::Zero();

    // TODO:: subscribe to a topic to receive the desired end effector pose
    // and set as the callback function. `computeDesiredJointState`.
    while (ros::ok())
    {

        std::cout << "Hello, world!" << std::endl;
    }

    return 0;
}

void computeDesiredJointState()
{

    /*
    inverseKinematicsSolver(robot, q, desired_pose);

    for (int i = 0; i < joint_pos.size(); i++)
    {
        jointState_msg_sim.position[i] = joint_pos[i];
        jointState_msg_sim.velocity[i] = 0.0;
        jointState_msg_sim.effort[i] = 0.0;
    }

    pub_des_jstate.publish(jointState_msg_sim);
    */
}