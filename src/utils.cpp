#include "utils.hpp"

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

SO3 SE3Operations::ro(SE3 T) {
    SO3 R;
    R << T(0, 0), T(0, 1), T(0, 2),
         T(1, 0), T(1, 1), T(1, 2),
         T(2, 0), T(2, 1), T(2, 2);
    return R;
}

VEC3 SE3Operations::tau(SE3 T) {
    VEC3 R;
    R << T(0, 3), 
         T(1, 3),
         T(2, 3);
    return R;
}

/**
 * @brief Converts a rotation matrix to an angle-axis representation
*/

VEC3 SE3Operations::rotmToAngleAxis(SO3 R) {
    VEC3 l;
    l << R(2, 1) - R(1, 2),
              R(0, 2) - R(2, 0),
              R(1, 0) - R(0, 1);
    VEC3 alphaR;
    double l_norm = l.norm();

    if (l_norm == 0) {
        // R is a diagonal vector
        if (R(0, 0) == 1 && R(1, 1) == 1 && R(2, 2) == 1) {
            // R is the identity matrix
            alphaR << 0, 0, 0;
        } else {
            alphaR << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
            alphaR = alphaR * M_PI / 2;
        }
    } else {
        alphaR = l * atan2(l_norm, R(0, 0) + R(1, 1) + R(2, 2) - 1) / l_norm;
    }
    return alphaR;
}

/**
 * @brief Computes the error vector between the desired and 
 * current end-effector poses.
*/

VEC6 LM::error(SE3 &T_curr, SE3 &T_des) {
    VEC3 tau_des = SE3Operations::tau(T_des);
    VEC3 tau_curr = SE3Operations::tau(T_curr);
    VEC3 alphaR = SE3Operations::rotmToAngleAxis(SE3Operations::ro(T_des) * SE3Operations::ro(T_curr).transpose());
    VEC6 e;
    e << tau_des - tau_curr, alphaR;
    return e;
}

/**
 * @brief Computes the gk term, which represents the direction
 * of the update for the joint configuration.
*/

VEC6 LM::gk(Eigen::Matrix<double, 6, 6> &J, VEC6 &e) {
    VEC6 g;
    g = J.transpose() * e;
    return g;
}

/**
 * @brief Computes the Ak matrix, which drives the update for
 * the joint coordinates.
*/

Eigen::Matrix<double, 6, 6> LM::Ak(Eigen::Matrix<double, 6, 6> &J, double E) {
    Eigen::Matrix<double, 6, 6> A;
    A = J.transpose() * J + E * lambda * Eigen::Matrix<double, 6, 6>::Identity();
    return A;
}

VEC6 SE3Operations::to6D(SE3 v){
    VEC3 tras = SE3Operations::tau(v);
    VEC3 rot = SE3Operations::rotmToAngleAxis(SE3Operations::ro(v));
    VEC6 ret;
    ret << tras, rot;
    return ret;
}

SE3 SE3Operations::getGripperPose(VEC3 pose, double yaw){
    SE3 T;
    T << cos(yaw), -sin(yaw), 0, pose(0),
         sin(yaw), cos(yaw), 0, pose(1),
         0, 0, 1, pose(2),
         0, 0, 0, 1;
    return T;
}
