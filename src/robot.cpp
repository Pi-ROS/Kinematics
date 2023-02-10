#include "../include/robot.hpp"
#include "../include/se3.hpp"
#include "../include/ros.hpp"
#include "../include/config.hpp"
#include <ros/ros.h>
#include <cmath>

constexpr double Robot::d1;
constexpr double Robot::a2;
constexpr double Robot::a3;
constexpr double Robot::d4;
constexpr double Robot::d5;
constexpr double Robot::d6;
constexpr double Robot::workingHeight;
constexpr double Controller::dt;
constexpr double Controller::T;

/**
 * @brief: converts an opening diameter to an actual joint configuration.
*/

VEC3 gripperOpeningToJointConfig(double d);

/**
 * @brief: Constructor for the Robot class.
 * @param: q - the joint state vector
 */

Robot::Robot(JointStateVector q)
{
    (this->q).resize(6);
    this->q = q;
    q_home << -0.32, -0.78, -2.56, -1.63, -1.57, 3.49;
    #if SOFT_GRIPPER
    q_gripper << 0.0, 0.0, 0.0;
    #else
    q_gripper << 1.8, 1.8, 1.8;
    #endif
    
    this->pose = SE3Operations::tau(this->forwardKinematics(q));
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

void Robot::move(VEC3 &pose){

    ROS_INFO_STREAM("STARTING spostamento piano");
    ros::Rate loop_rate(LOOP_FREQUENCY);
    VEC3 tau_des;
    tau_des << pose(0), pose(1), Robot::workingHeight;

    Controller::redundantController(*this, tau_des);
    ros::spinOnce();
    loop_rate.sleep();

    this->pose = pose;
    ros::Duration(0.5).sleep(); 
    ROS_INFO_STREAM("FINISH spostamento piano");
}

void Robot::descent(double h){
    ROS_INFO_STREAM("STARTING discensa");
    ros::Rate loop_rate(LOOP_FREQUENCY);
    VEC6 back = this->q;

    
    SE3 T_des;
    T_des <<    1, 0, 0, this->pose(0,0),
                0, 1, 0, this->pose(1,0),
                0, 0, 1, h,
                0, 0, 0, 1;
    VEC6 q_des;
    q_des = this->inverseKinematics(T_des);
    this->q = q_des;
    publishJoints(pub_jstate, q_des);
    ros::spinOnce();
    loop_rate.sleep();

    ROS_INFO_STREAM("FINISH discensa");

    ros::Duration(0.5).sleep();
    publishJoints(pub_jstate, back);
    ros::Duration(0.5).sleep(); 
}


void Robot::lineSearch(SE3 T_des){
    ros::Rate loop_rate(10);
    SE3 T_curr = forwardKinematics(q);

    JointStateVector initialError = LM::error(T_curr, T_des);
    VEC3 initialTransError;
    initialTransError << initialError(0), initialError(1), initialError(2);

    // Number of intermediate points
    int N = initialTransError.norm() * 7;
    VEC6 increment = initialError / N;
    VEC6 v_des = SE3Operations::to6D(T_curr) + increment;
    JointStateVector q_k = q;
    double E;
    VEC6 e;
    Eigen::Matrix<double, 6, 6> Jk;
    Eigen::Matrix<double, 6, 6> Ak;
    VEC6 gk;
    for (int j = 0; j < N; j++)
    {
        /* IK */
        int i = 0;
        
        while (i < LM::maxIterations)
        {

            // Current pose of the end-effector
            SE3 T_k = forwardKinematics(q_k);
            VEC6 v_k = SE3Operations::to6D(T_k);
            // Error vector
            e = v_des - v_k;
            // Error scalar
            double e_norm = e.norm();
            E = e_norm * e_norm * 0.5;

            if (E < LM::errTresh)
            {
                break;
            }

            Jk = jacobian(q_k);
            Ak = LM::Ak(Jk, E);
            gk = LM::gk(Jk, e);

            // Update the joint state vector
            q_k = q_k + Ak.inverse() * gk;
            
            i++;
        }
        
        v_des += increment;

        publishJoints(pub_jstate, q_k);
        ros::spinOnce();
        loop_rate.sleep();
    }
    q = q_k;
}



void Controller::redundantController(Robot &r, VEC3 &x_f){
    ros::Rate loop_rate(1/Controller::dt);
    VEC3 x_0 = SE3Operations::tau(r.forwardKinematics(r.q));
    VEC3 v_des = (x_f - x_0) / Controller::T;
    int N = Controller::T / Controller::dt;
    VEC6 q_k = r.q;
    VEC3 x_des;
    VEC3 x_e = x_0;
    // x_e: current position
    // x_des: desired (next) position
    for(int i = 1; i <= N; i++){
        x_des = x_e + v_des * Controller::dt;
        MAT6 jac = r.jacobian(q_k);
        VEC6 q0dot = Controller::computeQ0dot(q_k);
        VEC6 qdot = Controller::computeQdot(jac, q_k, x_e, x_des, v_des);
        q_k = q_k + qdot * Controller::dt;
        publishJoints(pub_jstate, q_k);
        ros::spinOnce();
        loop_rate.sleep();
        x_e = x_des;
    }
    r.q = q_k;
}

VEC6 Controller::computeQ0dot(VEC6 q){
    VEC6 result;
    double D1 = 1 / (Controller::q1max - Controller::q1min);
    double D2 = 1 / (Controller::q2max - Controller::q2min);
    double D3 = 1 / (Controller::q3max - Controller::q3min);
    double D4 = 1 / (Controller::q4max - Controller::q4min);
    double D5 = 1 / (Controller::q5max - Controller::q5min);
    double D6 = 1 / (Controller::q6max - Controller::q6min);

    double q1t = q(0) - Controller::q1avg;
    double q2t = q(1) - Controller::q2avg;
    double q3t = q(2) - Controller::q3avg;
    double q4t = q(3) - Controller::q4avg;
    double q5t = q(4) - Controller::q5avg;
    double q6t = q(5) - Controller::q6avg;

    result << D1 * D1 * q1t, D2 * D2 * q2t, D3 * D3 * q3t, D4 * D4 * q4t, D5 * D5 * q5t, D6 * D6 * q6t;
    result = result / 6.0;
    return result;
}


template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}


VEC6 Controller::computeQdot(MAT6 &Jac, VEC6 q, VEC3 xe, VEC3 xd, VEC3 vd){
    MAT36 Jtras;
    // translation
    Jtras << Jac(0,0), Jac(0,1), Jac(0,2), Jac(0,3), Jac(0,4), Jac(0,5),
             Jac(1,0), Jac(1,1), Jac(1,2), Jac(1,3), Jac(1,4), Jac(1,5),
             Jac(2,0), Jac(2,1), Jac(2,2), Jac(2,3), Jac(2,4), Jac(2,5);

    // rotation
    // Jtras << Jac(3,0), Jac(3,1), Jac(3,2), Jac(3,3), Jac(3,4), Jac(3,5),
    //          Jac(4,0), Jac(4,1), Jac(4,2), Jac(4,3), Jac(4,4), Jac(4,5),
    //          Jac(5,0), Jac(5,1), Jac(5,2), Jac(5,3), Jac(5,4), Jac(5,5);
    VEC6 q0dot = computeQ0dot(q);
    Eigen::MatrixXd JtransInv = pseudoinverse(Jtras);
    VEC6 qdot = JtransInv * vd + (Eigen::Matrix<double, 6, 6>::Identity() - JtransInv * Jtras) * q0dot;
    return qdot;
}


/*
void Controller::redundantControllerRotation(Robot &r, VEC3 &rpy_f){
    ros::Rate loop_rate(10);
    VEC3 rpy_0 = SE3Operations::rotmToEul(SE3Operations::ro(r.forwardKinematics(r.q)));
    VEC3 w_des = (rpy_f - rpy_0) / Controller::T;
    int N = Controller::T / Controller::dt;
    VEC6 q_k = r.q;
    VEC3 rpy_des;
    VEC3 rpy_e = rpy_0;
    // rpy_e: current position
    // rpy_des: desired (next) position
    for(int i = 1; i <= N; i++){
        rpy_des = rpy_e + w_des * Controller::dt;
        MAT6 jac = r.jacobian(q_k);
        VEC6 q0dot = Controller::computeQ0dot(q_k);
         VEC6 qdot = Controller::computeQdot(jac, q_k, rpy_e, rpy_des, w_des);
        q_k = q_k + qdot * Controller::dt;
        publishJoints(pub_jstate, q_k);
        ros::spinOnce();
        loop_rate.sleep();
        rpy_e = rpy_des;
    }
    r.q = q_k;
}
*/

void Robot::moveGripper(double d, int N, double dt) {
    VEC3 q_des = gripperOpeningToJointConfig(d);
    VEC3 incr = (q_des - q_gripper) / N;
    ros::Rate loop_rate(1/dt);

    VEC3 q_gripper_k = q_gripper + incr;
    for (int i=0; i<N; ++i) {
        publishJointsAndGripper(pub_jstate, q, q_gripper_k);
        q_gripper = q_gripper_k;
        q_gripper_k += incr;
        ros::spinOnce();
        loop_rate.sleep();
    }
}

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