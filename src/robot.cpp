#include "robot.hpp"
#include "utils.hpp"
#include "ros.hpp"
#include "config.hpp"
#include "link_attacher.hpp"
#include "controllers.hpp"
#include <ros/ros.h>
#include <cmath>

using namespace vector_field_controller;

constexpr double Robot::d1;
constexpr double Robot::a2;
constexpr double Robot::a3;
constexpr double Robot::d4;
constexpr double Robot::d5;
constexpr double Robot::d6;

/**
 * @brief: converts an opening diameter to an actual joint configuration.
*/

VEC3 gripperOpeningToJointConfig(double d);

Joints::Joints(VEC6 q, VEC3 gripper){
    this->update(q, gripper);
}

VEC6 Joints::q(){
    VEC6 q;
    q << this->shoulder_pan, this->shoulder_lift, this->elbow,
            this->wrist_1, this->wrist_2, this->wrist_3;
    return q;
    
}
VEC3 Joints::q_gripper(){
    VEC3 gripper;
    gripper << this->hand_1, this->hand_2, this->hand_3;
    return gripper;
    
}

void Joints::update(){
    VEC9 joints = readJoints();

    this->shoulder_pan = joints[0];
    this->shoulder_lift = joints[1];
    this->elbow = joints[2];
    this->wrist_1 = joints[3];
    this->wrist_2 = joints[4];
    this->wrist_3 = joints[5];
    this->hand_1 = joints[6];
    this->hand_2 = joints[7];
    this->hand_3 = joints[8];
}

void Joints::update(VEC6 q){
    this->shoulder_pan = q[0];
    this->shoulder_lift = q[1];
    this->elbow = q[2];
    this->wrist_1 = q[3];
    this->wrist_2 = q[4];
    this->wrist_3 = q[5];
}

void Joints::update(VEC3 gripper){
    this->hand_1 = gripper[0];
    this->hand_2 = gripper[1];
    this->hand_3 = gripper[2];
}

void Joints::update(VEC6 q, VEC3 gripper){
    this->shoulder_pan = q[0];
    this->shoulder_lift = q[1];
    this->elbow = q[2];
    this->wrist_1 = q[3];
    this->wrist_2 = q[4];
    this->wrist_3 = q[5];

    this->hand_1 = gripper[0];
    this->hand_2 = gripper[1];
    this->hand_3 = gripper[2];
}

Robot::Robot(VEC6 q)
{

    VEC3 gripper;

    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;

    #if SOFT_GRIPPER
    gripper << 0.0, 0.0, 0.0;
    #else
    gripper << 1.8, 1.8, 1.8;
    #endif

    this->joints = Joints(q, gripper);
    this->pose = SE3Operations::tau(this->forwardKinematics(q));
    
}


/**
 * @brief: Transformation matrixes
 * @param: theta - the joint angle
 */

SE3 Robot::T01(double theta1)
{
    SE3 tmp;
    tmp << cos(theta1), -sin(theta1), 0, 0,
           sin(theta1), cos(theta1), 0, 0,
           0, 0, 1, Robot::d1,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T12(double theta2)
{
    SE3 tmp;
    tmp << cos(theta2), -sin(theta2), 0, 0,
           0, 0, -1, 0,
           sin(theta2), cos(theta2), 0, 0,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T23(double theta3)
{
    SE3 tmp;
    tmp << cos(theta3), -sin(theta3), 0, Robot::a2,
           sin(theta3), cos(theta3), 0, 0,
           0, 0, 1, 0,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T34(double theta4)
{
    SE3 tmp;
    tmp << cos(theta4), -sin(theta4), 0, Robot::a3,
           sin(theta4), cos(theta4), 0, 0,
           0, 0, 1, Robot::d4,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T45(double theta5)
{
    SE3 tmp;
    tmp << cos(theta5), -sin(theta5), 0, 0,
           0, 0, -1, -Robot::d5,
           sin(theta5), cos(theta5), 0, 0,
           0, 0, 0, 1;
    return tmp;
}
SE3 Robot::T56(double theta6)
{
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

SE3 Robot::forwardKinematics(VEC6 &q)
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

MAT6 Robot::jacobian(VEC6 q)
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

VEC6 Robot::inverseKinematics(SE3 &T_des)
{
    int i = 0;
    VEC6 q_k = this->joints.q();

    while (i < LM::maxIterations)
    {
        // Current pose of the end-effector
        SE3 T_k = forwardKinematics(q_k);
        // Error vector
        VEC6 e = LM::error(T_k, T_des);
        // Error scalar
        double e_norm = e.norm();
        double E = e_norm * e_norm * 0.5;

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
    
    normalize(q_k);
    return q_k;
}

void Robot::move(SE3 &T_des){
    ROS_INFO_STREAM("START: planar motion");
    ros::Rate loop_rate(LOOP_FREQUENCY);

    vectorFieldController(*this, T_des);
    loop_rate.sleep();

    this->joints.update();
    ROS_INFO_STREAM("FINISH: planar motion");
}

void Robot::descent(SE3 &T_des, bool pick, ros::ServiceClient &gripperClient){
    ros::Rate loop_rate(LOOP_FREQUENCY);
    VEC6 q0 = this->joints.q();
    VEC3 q_gripper = this->joints.q_gripper();
    VEC6 q_des;
    
    ROS_INFO_STREAM("START: descent");
    SE3 T_curr = this->forwardKinematics(q0);
    SE3 T_des1 = T_des;
    T_des1(0, 3) = T_curr(0, 3);
    T_des1(1, 3) = T_curr(1, 3);
    T_des1(2, 3) = T_curr(2, 3);
    q_des = this->inverseKinematics(T_des1);
    velocityController(*this, DT, VELOCITY, q_des, false);

    q_des = this->inverseKinematics(T_des);
    velocityController(*this, DT, VELOCITY, q_des, false);
    ROS_INFO_STREAM("FINISH: descent");
   
    if(pick)
        this->moveGripper(gripperClient, GRIPPER_OPENING, 10, 0.1); 
    else
        this->moveGripper(gripperClient, 180, 10, 0.1);
        
    loop_rate.sleep();
    
    ROS_INFO_STREAM("START: ascent");
    velocityController(*this, DT, VELOCITY, q0, true);
    ROS_INFO_STREAM("FINISH: ascent");
}

#if SIMULATION
void Robot::moveGripper(ros::ServiceClient &gripperClient, double d, int N, double dt) {
    // gripper service is not used here
    VEC3 q_des = gripperOpeningToJointConfig(d);
    VEC3 q_gripper = this->joints.q_gripper();
    VEC6 q = this->joints.q();

    VEC3 incr = (q_des - q_gripper) / N;
    ros::Rate loop_rate(1/dt);

    VEC3 q_gripper_k = q_gripper + incr;
    for (int i=0; i<N; ++i) {
        publishJoints(pub_jstate, q, q_gripper_k);
        q_gripper = q_gripper_k;
        q_gripper_k += incr;
        loop_rate.sleep();
    }
    this->joints.update();
}
#else
void Robot::moveGripper(ros::ServiceClient &gripperClient, double d, int N, double dt) {
    // N and dt are not used in the real robot, gripper service is used instead
    ros_impedance_controller::generic_float gripper_srv;
    gripper_srv.request.data = d;
    if (!gripperClient.call(gripper_srv) || !gripper_srv.response.ack) {
        ROS_INFO_STREAM("Gripper service call failed");
        exit(0);
    }
}
#endif

#if SOFT_GRIPPER
VEC3 gripperOpeningToJointConfig(double d)
{
    double D0 = 40;
    double L = 60;
    double opening = atan2(0.5*(d - D0), L);
    VEC3 q;
    q << opening, opening, 0;
    return q;
}
#else
VEC3 gripperOpeningToJointConfig(double d)
{
    double opening = (d - 22) / 108 * -M_PI + M_PI;
    VEC3 q;
    q << opening, opening, opening;
    return q;
}
#endif