#ifndef SE3_HPP
#define SE3_HPP

#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 6> Jacobian;
typedef Eigen::Matrix<double, 4, 4> SE3;
typedef Eigen::Matrix<double, 3, 3> SO3;
typedef Eigen::Matrix<double, 3, 1> VEC3;
typedef Eigen::Matrix<double, 6, 1> JointStateVector;

class SE3Operations{
    public:
        static SE3 Rx(double th);
        static SE3 Ry(double th);
        static SE3 Rz(double th);
        static SE3 Tx(double d);
        static SE3 Ty(double d);
        static SE3 Tz(double d);
        static SO3 rot(SE3 T);
        static VEC3 tra(SE3 T);
        
};


// // Derivative of a pure translation about the x-axis, with respect to
// // the joint coordinate
// SE3 dTx(double d);
// // Derivative of a pure translation about the y-axis, with respect to
// // the joint coordinate
// SE3 dTy(double d);
// // Derivative of a pure translation about the z-axis, with respect to
// // the joint coordinate
// SE3 dTz(double d);
// // Derivative of a pure rotation about the x-axis, with respect to
// // the joint coordinate
// SE3 dRx(double th);
// // Derivative of a pure rotation about the y-axis, with respect to
// // the joint coordinate
// SE3 dRy(double th);
// // Derivative of a pure rotation about the z-axis, with respect to
// // the joint coordinate
// SE3 dRz(double th);
// // Inverse skew operation. Takes a 3x3 skew-symmetric matrix and returns
// // a 3x1 vector.
// Eigen::Matrix<double, 3, 1> invskew(Eigen::Matrix<double, 3, 3> &S);
// Extract the rotation matrix from a homogeneous transformation matrix

#endif