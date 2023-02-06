#include "kinematics.hpp"

int main(int argc, char **argv)
{
    JointStateVector joint_pos;

    ros::init(argc, argv, "direct_kinematics");

    ROS_INFO_STREAM("STARTING KINEMATICS NODE");
    if (argc == 2)
    {
        DEBUG = true;
        ROS_INFO_STREAM("--- DEBUG MODE ---");
    }

    ros::NodeHandle node;
    // ros::Publisher pub_jstate;
    // ros::Subscriber sub_jstate;
    if (DEBUG)
    {
        pub_jstate = node.advertise<std_msgs::Float64MultiArray>(debug_topic, 1);
        sub_jstate = node.subscribe(debug_topic, 1000, readJointsDebug);
    }
    else
    {
        pub_jstate = node.advertise<std_msgs::Float64MultiArray>(joint_state_publisher_topic, 1);
        sub_jstate = node.subscribe(joint_state_subscriber_topic, 1000, readJoints);
    }


    ros::Rate loop_rate(LOOP_FREQUENCY);

    VEC6 q0;
    q0 << 5.5127358094431145e-05, -0.0001212409520094937, 9.891741610346116e-05, 0.00013209079602738427, 4.505457065562268e-05, 6.162511098306567e-05;
    Robot ur5(q0);

    joint_pos.resize(6);
    data_read.resize(6);

    /* --------- DEBUG CODE --------- */
    if (DEBUG)
    {
        // home position
        joint_pos << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;

        SE3 ee = ur5.forwardKinematics(joint_pos);
        VEC3 ee_pos = SE3Operations::tau(ee);
        SO3 ee_rot = SE3Operations::ro(ee);
        ROS_INFO_STREAM("Joints configuration:");
        ROS_INFO_STREAM(joint_pos(0) << " " << joint_pos(1) << " " << joint_pos(2) << " " << joint_pos(3) << " " << joint_pos(4) << " " << joint_pos(5));
        ROS_INFO_STREAM("EE positions: ");
        ROS_INFO_STREAM(ee_pos(0) << " " << ee_pos(1) << " " << ee_pos(2));
        ROS_INFO_STREAM("EE rotation: ");
        ROS_INFO_STREAM(ee_rot(0) << " " << ee_rot(1) << " " << ee_rot(2));
        ROS_INFO_STREAM(ee_rot(3) << " " << ee_rot(4) << " " << ee_rot(5));
        ROS_INFO_STREAM(ee_rot(6) << " " << ee_rot(7) << " " << ee_rot(8));
        ROS_INFO_STREAM("--------------------");

        double x = 0.01;

        while (ros::ok())
        {
            /* CODE HERE */
            // test movement
            x = x + 0.01;
            joint_pos << x, x, x, x, x, x;

            /* CODE fFOR PUBLISHING ON pub_jstate*/
            publishJoints(pub_jstate, joint_pos);
            ros::spinOnce();
            loop_rate.sleep();
            ROS_INFO_STREAM("Data read: " << data_read);
        }
    }
    /* --------- END DEBUG CODE --------- */
    /* --------- REAL CODE --------- */
    else
    {
        // disp = [0.2; 0.5; 0.7];
        // rot = [cos(th) -sin(th) 0;
        //     sin(th) cos(th)  0;
        //     0         0      1];

        // Tbe = [rot, disp; 0 0 0 1];

        JointStateVector q_home;
        q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
        ur5.move(0.01, 0.6, q_home);
       

        ROS_INFO_STREAM("HO FINITO");
        
        while (ros::ok())
        {
            loop_rate.sleep();
            //ROS_INFO_STREAM("Destination reached");
            ros::spinOnce();
            //ROS_INFO_STREAM("Data read: " << data_read);
        }
    }
    /* --------- END REAL CODE --------- */

    return 0;
}