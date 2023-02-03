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

/*-------------*/
/**
 * @brief: Computation of the Jacobian matrix for the transformation from
 * the base frame to the end effector frame, with respect to the joints coordinates.
*/

Jacobian Robot::jacobian() {
    /**
     * Rows 3-5 of the Jacobian are 0 because there are no prismatic joints. 
     * ci: cos(thi). si: sin(thi). rij = rot(T06)[i, j].
     * pi: tra(T06)[i]. di, ai: DH parameters.
     * 
     * J =  [   0,             s1,                              s1,                      s1,                     c1s234, r13 ]
     *      [   0,            -c1,                             -c1,                     -c1,                     s1s234, r23 ]
     *      [   1,              0,                               0,                       0,                      -c234, r33 ]
     *      [ -py,   -c1(pz - d1),   c1(s234s5d6 + c234d5 - s23a3),   c1(s234s5d6 + c234d5),       -d6(s1s5 + c1c234c5),   0 ]
     *      [  px,   -s1(pz - d1),   s1(s234s5d6 + c234d5 - s23a3),   s1(s234s5d6 + c234d5),        d6(c1s5 - c234c5s1),   0 ]
     *      [   0,    s1py + c1px,      -c234s5d6 + s234d5 + c23a3,      -c234s5d6 + s234d5,                  -c5s234d6,   0 ]
    */  

    Jacobian J;
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
    double r13 = -c1*c234*s5 + c5*s1;
    double r23 = -s1*c234*s5 - c1*c5;
    double r33 = -s5*s234;
    double px = r13*d6 + c1*(s234*d5 + c23*a3 + c2*a2) + s1*d4;
    double py = r23*d6 + s1*(s234*d5 + c23*a3 + c2*a2) - c1*d4;
    double pz = r33*d6 - c234*d5 + s23*a3 + s2*a2 + d1;

    J << -py, -c1*(pz - d1), c1*(s234*s5*d6 + c234*d5 - s23*a3), c1*(s234*s5*d6 + c234*d5), -d6*(s1*s5 + c1*c234*c5), 0,
         px, -s1*(pz - d1), s1*(s234*s5*d6 + c234*d5 - s23*a3), s1*(s234*s5*d6 + c234*d5), d6*(c1*s5 - c234*c5*s1), 0,
         0, s1*py + c1*px, -c234*s5*d6 + s234*d5 + c23*a3, -c234*s5*d6 + s234*d5, -c5*s234*d6, 0,
         0, s1, s1, s1, c1*s234, r13,
         0, -c1, -c1, -c1, s1*s234, r23,
         1, 0, 0, 0, -c234, r33;
    return J;
}