#ifndef CONTROLLERS_HPP
#define CONTROLLERS_HPP

#include <cmath>
#include <Eigen/Dense>
#include "config.hpp"
#include "robot.hpp"

namespace vector_field_controller {
    // Potential field parameters
    constexpr double Lambda = VELOCITY;
    constexpr double r0 = 0.25;
    constexpr double N0 = 200;
    constexpr double ErrThresh = 0.02;
    
    VEC3 vectorField(VEC3 p_curr, VEC3 p_f);
    double scalarRotVelocity(VEC3 e, int iter);
    double scalarVelocity(VEC3 e, int iter);
    VEC3 rotationalVelocity(VEC3 e, int iter);
    VEC3 velocity(VEC3 e, VEC3 p_curr, VEC3 p_f, int iter);
    VEC6 qDot(Robot &r, VEC6 q, VEC6 e, VEC3 p_f, int iter);
    void vectorFieldController(Robot &r, SE3 &T_des);
}

namespace redundant_controller {
    constexpr double q1min = 0;
    constexpr double q1max = M_PI;
    constexpr double q1avg = M_PI / 2.0;
    constexpr double q2min = 0;
    constexpr double q2max = M_PI;
    constexpr double q2avg = M_PI / 2.0;
    constexpr double q3min = - M_PI;
    constexpr double q3max = 0;
    constexpr double q3avg = - M_PI / 2.0;
    constexpr double q4min = - M_PI / 2.0;
    constexpr double q4max = 0;
    constexpr double q4avg = M_PI / 2.0;
    constexpr double q5min = - M_PI / 2.0;
    constexpr double q5max = 0;
    constexpr double q5avg = M_PI / 2.0;
    constexpr double q6min = - M_PI / 2.0;
    constexpr double q6max = 0;
    constexpr double q6avg = M_PI / 2.0;
    constexpr double T = 2;

    VEC6 computeQ0dot(VEC6 q);
    VEC6 computeQdot(MAT6 &Jac, VEC6 q, VEC3 xe, VEC3 xd, VEC3 vd);
    void redundantController(Robot &r, VEC3 &x_f);
};

void velocityController(Robot &r, double dt, double v_des, VEC6 q_f, bool ascent = false);

#endif