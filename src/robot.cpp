#include "../include/robot.hpp"
#include "../include/se3.hpp"
#include "../include/ros.hpp"
#include <cmath>

/**
 * @brief: Constructor for the Robot class.
 * @param: q - the joint state vector
 */

Robot::Robot(JointStateVector q)
{
    (this->q).resize(6);
    this->q = q;
}

/**
 * @brief: Transformation matrixes
 * @param: theta - the joint angle
 */
SE3 Robot::T01(double theta1)
{
    //return SE3Operations::Tz(Robot::d1) * SE3Operations::Rz(theta1);
    SE3 tmp;
    tmp << cos(theta1), -sin(theta1), 0, 0,
           sin(theta1), cos(theta1), 0, 0,
           0, 0, 1, Robot::d1,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T12(double theta2)
{
    //return SE3Operations::Rx(M_PI_2) * SE3Operations::Rz(theta2);
    SE3 tmp;
    tmp << cos(theta2), -sin(theta2), 0, 0,
           0, 0, -1, 0,
           sin(theta2), cos(theta2), 0, 0,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T23(double theta3)
{
    //return SE3Operations::Tz(Robot::a2) * SE3Operations::Rz(theta3);
    SE3 tmp;
    tmp << cos(theta3), -sin(theta3), 0, Robot::a2,
           sin(theta3), cos(theta3), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T34(double theta4)
{
    //return SE3Operations::Tx(Robot::a3) * SE3Operations::Tz(Robot::d4) * SE3Operations::Rz(theta4);
    SE3 tmp;
    tmp << cos(theta4), -sin(theta4), 0, Robot::a3,
           sin(theta4), cos(theta4), 0, 0,
           0, 0, 1, Robot::d4,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T45(double theta5)
{
    //return SE3Operations::Ty(Robot::d5) * SE3Operations::Rz(M_PI_2) * SE3Operations::Rz(theta5);
    SE3 tmp;
    tmp << cos(theta5), -sin(theta5), 0, 0,
           0, 0, -1, -Robot::d5,
           sin(theta5), cos(theta5), 0, 0,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T56(double theta6)
{
    //return SE3Operations::Ty(Robot::d6) * SE3Operations::Rz(-M_PI_2) * SE3Operations::Rz(theta6);
    SE3 tmp;
    tmp << cos(theta6), -sin(theta6), 0, 0,
           0, 0, 1, Robot::d6,
           -sin(theta6), -cos(theta6), 0, 0,
           0, 0, 0, 1;
    return tmp;
}

/**
 * @brief: Forward kinematics
 * @param: q - the joint state vector
 */

SE3 Robot::forwardKinematics(JointStateVector &q)
{
    // Transformation matrixes
    SE3 t01 = T01(q(0, 0));
    SE3 t12 = T12(q(1, 0));
    SE3 t23 = T23(q(2, 0));
    SE3 t34 = T34(q(3, 0));
    SE3 t45 = T45(q(4, 0));
    SE3 t56 = T56(q(5, 0));
    // Transformation matrixes from the base frame to the end-effector
    SE3 t06 = t01 * t12 * t23 * t34 * t45 * t56;
    return t06;
}

/**
 * @brief: Jacobian of the end-effector transformation with respect to
 * the current joints coordinates.
 */

Eigen::Matrix<double, 6, 6> Robot::jacobian(VEC6 q)
{
    Eigen::Matrix<double, 6, 6> J;
    double s1 = sin(q(0, 0));
    double c1 = cos(q(0, 0));
    double c2 = cos(q(1, 0));
    double s2 = sin(q(1, 0));
    double s5 = sin(q(4, 0));
    double c5 = cos(q(4, 0));
    double s234 = sin(q(1, 0) + q(2, 0) + q(3, 0));
    double c23 = cos(q(1, 0) + q(2, 0));
    double s23 = sin(q(1, 0) + q(2, 0));
    double c234 = cos(q(1, 0) + q(2, 0) + q(3, 0));
    double r13 = -c1 * c234 * s5 + c5 * s1;
    double r23 = -s1 * c234 * s5 - c1 * c5;
    double r33 = -s5 * s234;
    double px = r13 * d6 + c1 * (s234 * d5 + c23 * a3 + c2 * a2) + s1 * d4;
    double py = r23 * d6 + s1 * (s234 * d5 + c23 * a3 + c2 * a2) - c1 * d4;
    double pz = r33 * d6 - c234 * d5 + s23 * a3 + s2 * a2 + d1;

    J << -py, -c1 * (pz - d1), c1 * (s234 * s5 * d6 + c234 * d5 - s23 * a3), c1 * (s234 * s5 * d6 + c234 * d5), -d6 * (s1 * s5 + c1 * c234 * c5), 0,
        px, -s1 * (pz - d1), s1 * (s234 * s5 * d6 + c234 * d5 - s23 * a3), s1 * (s234 * s5 * d6 + c234 * d5), d6 * (c1 * s5 - c234 * c5 * s1), 0,
        0, s1 * py + c1 * px, -c234 * s5 * d6 + s234 * d5 + c23 * a3, -c234 * s5 * d6 + s234 * d5, -c5 * s234 * d6, 0,
        0, s1, s1, s1, c1 * s234, r13,
        0, -c1, -c1, -c1, s1 * s234, r23,
        1, 0, 0, 0, -c234, r33;
    return J;
}

/**
 * @brief Inverse kinematics implementation based on the LM algorithm.
 */

JointStateVector Robot::inverseKinematics(SE3 &T_des)
{
    int i = 0;
    JointStateVector q_k = q;

    while (i < LM::maxIterations)
    {

        // Current pose of the end-effector
        SE3 T_k = forwardKinematics(q_k);
        // Error vector
        VEC6 e = LM::error(T_k, T_des);
        // Error scalar
        double E = e.norm() * 0.5;

        if (E < LM::errTresh)
        {
            break;
        }

        Eigen::Matrix<double, 6, 6> Jk = jacobian(q_k);
        Eigen::Matrix<double, 6, 6> Ak = LM::Ak(Jk, E);
        VEC6 gk = LM::gk(Jk, e);

        // Update the joint state vector
        q_k = q_k + Ak.inverse() * gk;
        ROS_INFO_STREAM("iter " << i << "\nq_k: " << q_k.transpose());
        ROS_INFO_STREAM("e_k: " << e.transpose() << "\n-------------------");
        i++;
    }
    ROS_INFO_STREAM("iter: " << i);
    return q_k;
}

void Robot::homingProcedure(double dt, double v_des, VEC6 q_des)
{   
    ROS_INFO_STREAM("Starting to move");
    double v_ref = 0.0;
    JointStateVector q_k = q;
    JointStateVector e;
    double e_norm;
    ros::Rate rate(1 / dt);
    while (true)
    {
        e = q_des - q_k;
        e_norm = e.norm();
        if (e_norm != 0.0)
        {
            v_ref += 0.005 * (v_des - v_ref);
            q_k += dt * v_ref * e / e_norm;
            publishJoints(pub_jstate, q_k);
            ros::spinOnce();
            rate.sleep();
        }
        rate.sleep();
        if (e_norm < 0.001)
        {
            ROS_INFO_STREAM("Reached the desired position");
            break;
        }
    }
}

void Robot::lineSearch(SE3 T_des){
    ros::Rate loop_rate(10);
    SE3 T_curr = forwardKinematics(q);
    JointStateVector e = LM::error(T_curr, T_des);
    VEC3 tau_e;
    tau_e << e(0), e(1), e(2);
    int N = tau_e.norm() * 10;
    //int N = 1;
    VEC6 increment = e / N;
    VEC6 v_des = SE3Operations::to6D(T_curr) + increment;
    JointStateVector q_k = q;
    for (int j = 0; j < N; j++)
    {
        /* IK */
        int i = 0;

        while (i < LM::maxIterations)
        {

            // Current pose of the end-effector
            SE3 T_k = forwardKinematics(q_k);
            // Error vector
            VEC6 v_k = SE3Operations::to6D(T_k);
            VEC6 e = v_des - v_k;
            // Error scalar
            double E = e.norm() * 0.5;

            if (E < LM::errTresh)
            {
                break;
            }

            Eigen::Matrix<double, 6, 6> Jk = jacobian(q_k);
            Eigen::Matrix<double, 6, 6> Ak = LM::Ak(Jk, E);
            VEC6 gk = LM::gk(Jk, e);

            // Update the joint state vector
            q_k = q_k + Ak.inverse() * gk;
            
            i++;
        }
        /*---*/
        ROS_INFO_STREAM("iter " << i << "\nq_k: " << q_k.transpose());
        ROS_INFO_STREAM("e_k: " << e.transpose() << "\n-------------------");
        publishJoints(pub_jstate, q_k);
        ros::spinOnce();
        loop_rate.sleep();
        v_des += increment;
    }
    q = q_k;
}

void Controller::redundantController(){

}

