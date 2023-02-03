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
