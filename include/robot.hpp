#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include <Eigen/Dense>
// #include "kinematics.hpp"
#include "se3.hpp"

class Robot
{

private:
    double d1;
    double a2;
    double a3;
    double d4;
    double d5;
    double d6;

    // double d1 = 0.163;
    // double Robot::a2 = -0.42500;
    // static double Robot::a3 = -0.39225;
    // static double Robot::d4 = 0.134;
    // static double Robot::d5 = 0.100;
    // static double Robot::d6 = 0.100;

public:
    // Joint state
    JointStateVector q;
    // Constructor
    Robot(JointStateVector q);
    // Transformation matrix from the base frame to frame 1
    SE3 T01(double theta1);
    // Tranformation matrix from frame 1 to frame 2
    SE3 T12(double theta2);
    // Transformation matrix from frame 2 to frame 3
    SE3 T23(double theta3);
    // Transformation matrix from frame 3 to frame 4
    SE3 T34(double theta4);
    // Transformation matrix from frame 4 to frame 5
    SE3 T45(double theta5);
    // Transformation matrix from frame 5 to frame 6
    SE3 T56(double theta6);
    // // Transformation matrix from frame 0 to frame 2
    // SE3 T02();
    // // Transformation matrix from frame 0 to frame 3
    // SE3 T03();
    // // Transformation matrix from frame 0 to frame 4
    // SE3 T04();
    // // Transformation matrix from frame 0 to frame 5
    // SE3 T05();
    // // Transformation matrix from frame 0 to frame 6
    // SE3 T06();
    // Jacobian of the end-effector transformation with respect to
    // the joints coordinates
    Jacobian jacobian();

    /**
     * @brief Forward kinematics
     *
     */
    SE3 forwardKinematics(JointStateVector q);
};

#endif