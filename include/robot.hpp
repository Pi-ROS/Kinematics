#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include <Eigen/Dense>
#include "kinematics.hpp"

typedef Eigen::Matrix<double, 6, 6> Jacobian;
typedef Eigen::Matrix<double, 6, 1> JointStateVector;
typedef Eigen::Matrix<double, 4, 4> SE3;

class Robot {

private:
    // Transformation matrix for a pure translation about the x-axis
    SE3 Tx(double d);
    // Transformation matrix for a pure translation about the y-axis
    SE3 Ty(double d);
    // Transformation matrix for a pure translation about the z-axis
    SE3 Tz(double d);
    // Transformation matrix for a pure rotation about the x-axis
    SE3 Rx(double th);
    // Transformation matrix for a pure rotation about the y-axis
    SE3 Ry(double th);
    // Transformation matrix for a pure rotation about the z-axis
    SE3 Rz(double th);

    // Derivative of a pure translation about the x-axis, with respect to 
    // the joint coordinate
    SE3 dTx(double d);
    // Derivative of a pure translation about the y-axis, with respect to
    // the joint coordinate
    SE3 dTy(double d);
    // Derivative of a pure translation about the z-axis, with respect to
    // the joint coordinate
    SE3 dTz(double d);
    // Derivative of a pure rotation about the x-axis, with respect to
    // the joint coordinate
    SE3 dRx(double th);
    // Derivative of a pure rotation about the y-axis, with respect to
    // the joint coordinate
    SE3 dRy(double th);
    // Derivative of a pure rotation about the z-axis, with respect to
    // the joint coordinate
    SE3 dRz(double th);
    // Inverse skew operation. Takes a 3x3 skew-symmetric matrix and returns
    // a 3x1 vector.
    Eigen::Matrix<double, 3, 1> invskew(Eigen::Matrix<double, 3, 3> &S);

public:
    // Joint state
    JointStateVector q;
    // Constructor
    Robot(JointStateVector q);
    // Transformation matrix from the base frame to frame 1
    SE3 T01();
    // Tranformation matrix from frame 1 to frame 2
    SE3 T12();
    // Transformation matrix from frame 2 to frame 3
    SE3 T23();
    // Transformation matrix from frame 3 to frame 4
    SE3 T34();
    // Transformation matrix from frame 4 to frame 5
    SE3 T45();
    // Transformation matrix from frame 5 to frame 6
    SE3 T56();
    // Transformation matrix from frame 0 to frame 2
    SE3 T02();
    // Transformation matrix from frame 0 to frame 3
    SE3 T03();
    // Transformation matrix from frame 0 to frame 4
    SE3 T04();
    // Transformation matrix from frame 0 to frame 5
    SE3 T05();
    // Transformation matrix from frame 0 to frame 6
    SE3 T06();
    // Jacobian of the end-effector transformation with respect to
    // joint coordinates
    Jacobian jacobian();
}

#endif