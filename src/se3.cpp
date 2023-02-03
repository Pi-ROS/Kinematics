#include "../include/se3.hpp"
#include <cmath>


SE3 SE3Operations::Rx(double th) {
    SE3 R;
    R << 1, 0, 0, 0,
         0, cos(th), -sin(th), 0,
         0, sin(th), cos(th), 0,
         0, 0, 0, 1;
    return R;
}

SE3 SE3Operations::Ry(double th) {
    SE3 R;
    R << cos(th), 0, sin(th), 0,
         0, 1, 0, 0,
         -sin(th), 0, cos(th), 0,
         0, 0, 0, 1;
    return R;
}

SE3 SE3Operations::Rz(double th) {
    SE3 R;
    R << cos(th), -sin(th), 0, 0,
         sin(th), cos(th), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return R;
}

SE3 SE3Operations::Tx(double d) {
    SE3 T;
    T << 1, 0, 0, d,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return T;
}

SE3 SE3Operations::Ty(double d) {
    SE3 T;
    T << 1, 0, 0, 0,
         0, 1, 0, d,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return T;
}

SE3 SE3Operations::Tz(double d) {
    SE3 T;
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, d,
         0, 0, 0, 1;
    return T;
}

SO3 SE3Operations::rot(SE3 T) {
    SO3 R;
    R << T(0, 0), T(0, 1), T(0, 2),
         T(1, 0), T(1, 1), T(1, 2),
         T(2, 0), T(2, 1), T(2, 2);
    return R;
}

VEC3 SE3Operations::tra(SE3 T) {
    VEC3 R;
    R << T(0, 3), 
         T(1, 3),
         T(2, 3);
    return R;
}


/*-------------*/
// /**
//  * @brief: Computation of the Jacobian matrix for the transformation from
//  * the base frame to the end effector frame, with respect to the joints coordinates.
// */

// Jacobian Robot::Jacobian() {
//     // Angular velocities
//     Eigen::Matrix<double, 3, 1> Jw1, Jw2, Jw3, Jw4, Jw5, Jw6;
//     // Linear velocities
//     Eigen::Matrix<double, 3, 1> Jv1, Jv2, Jv3, Jv4, Jv5, Jv6;
//     Jacobian J;

//     // TODO: compute the jacobian for the UR5
//     return J;
// }



// /**
//  * @brief: Derivative of the transformation matrix for a pure translation
//  * about the x-axis.
//  * @param: d - the joint coordinate
//  */
// SE3 Robot::dTx(double d) {
//     SE3 dTx;
//     dTx << 0, 0, 0, 1,
//            0, 0, 0, 0,
//            0, 0, 0, 0,
//            0, 0, 0, 0;
//     return dTx;
// }

// /**
//  * @brief: Derivative of the transformation matrix for a pure translation
//  * about the y-axis.
//  * @param: d - the joint coordinate
//  */

// SE3 Robot::dTy(double d) {
//     SE3 dTy;
//     dTy << 0, 0, 0, 0,
//            0, 0, 0, 1,
//            0, 0, 0, 0,
//            0, 0, 0, 0;
//     return dTy;
// }

// /**
//  * @brief: Derivative of the transformation matrix for a pure translation
//  * about the z-axis.
//  * @param: d - the joint coordinate
//  */

// SE3 Robot::dTz(double d) {
//     SE3 dTz;
//     dTz << 0, 0, 0, 0,
//            0, 0, 0, 0,
//            0, 0, 0, 1,
//            0, 0, 0, 0;
//     return dTz;
// }

// /**
//  * @brief: Derivative of the transformation matrix for a pure rotation
//  * about the x-axis.
//  * @param: th - the joint coordinate
//  */

// SE3 Robot::dRx(double th) {
//     SE3 dRx;
//     dRx << 0, 0, 0, 0,
//            0, 0, -1, 0,
//            0, 1, 0, 0,
//            0, 0, 0, 0;
//     return dRx * Rx(th);
// }

// /**
//  * @brief: Derivative of the transformation matrix for a pure rotation
//  * about the y-axis.
//  * @param: th - the joint coordinate
//  */

// SE3 Robot::dRy(double th) {
//     SE3 dRy;
//     dRy << 0, 0, 1, 0,
//            0, 0, 0, 0,
//            -1, 0, 0, 0,
//            0, 0, 0, 0;
//     return dRy * Ry(th);
// }

// /**
//  * @brief: Derivative of the transformation matrix for a pure rotation
//  * about the z-axis.
//  * @param: th - the joint coordinate
//  */

// SE3 Robot::dRz(double th) {
//     SE3 dRz;
//     dRz << 0, -1, 0, 0,
//            1, 0, 0, 0,
//            0, 0, 0, 0,
//            0, 0, 0, 0;
//     return dRz * Rz(th);
// }

// /**
//  * @brief: Inverse skew operation. Takes a 3x3 skew-symmetric matrix and
//  * returns a 3x1 vector.
//  * @param: S - the skew-symmetric matrix
//  */

// Eigen::Matrix<double, 3, 1> Robot::invskew(Eigen::Matrix<double, 3, 3> &S) {
//     Eigen::Matrix<double, 3, 1> v;
//     v << S(2, 1), S(0, 2), S(1, 0);
//     return v;
// }