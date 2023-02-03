#include "../include/robot.hpp"
#include "../include/se3.hpp"
#include <cmath>

/**
 * @brief: Constructor for the Robot class.
 * @param: q - the joint state vector
 */

Robot::Robot(JointStateVector q){
    (this->q).resize(6);
    this->q = q;

}

/**
 * @brief: Transformation matrixes
 * @param: theta - the joint angle
 */
SE3 Robot::T01(double theta1) {
    return SE3Operations::Tz(Robot::d1) * SE3Operations::Rx(theta1);
}
SE3 Robot::T12(double theta2) {
    return SE3Operations::Rx(M_PI_2) * SE3Operations::Rz(theta2);
}
SE3 Robot::T23(double theta3) {
    return SE3Operations::Tz(Robot::a2) * SE3Operations::Rz(theta3);
}
SE3 Robot::T34(double theta4) {
    return SE3Operations::Tx(Robot::a3)* SE3Operations::Tz(Robot::d4) * SE3Operations::Rz(theta4);
}
SE3 Robot::T45(double theta5) {
    return SE3Operations::Ty(Robot::d5) * SE3Operations::Rz(M_PI_2) * SE3Operations::Rz(theta5);
}
SE3 Robot::T56(double theta6) {
    return SE3Operations::Ty(Robot::d6) * SE3Operations::Rz(- M_PI_2) * SE3Operations::Rz(theta6);
}

SE3 Robot::forwardKinematics(JointStateVector q){
    // Transformation matrixes
    SE3 t01 = T01(q(0,0));
    SE3 t12 = T12(q(1,0));
    SE3 t23 = T23(q(2,0));
    SE3 t34 = T34(q(3,0));
    SE3 t45 = T45(q(4,0));
    SE3 t56 = T56(q(5,0));
    // Transformation matrixes from the base frame to the end-effector
    SE3 t06 = t01 * t12 * t23 * t34 * t45 * t56;
    return t06;
}