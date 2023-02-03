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

#endif