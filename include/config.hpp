#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>
#include "ros.hpp"
#define SIMULATION false
#define USE_GRIPPER true
#define SOFT_GRIPPER false
#define TASK0 false
#define DT 0.001

#if SIMULATION
#define VELOCITY 2
#else
#define VELOCITY 0.2
#endif

static Eigen::Matrix<double, 6, 1> Q_HOME;

inline void initConfig() {
    Q_HOME << -0.32, -0.78, -2.56,-1.63, -1.57, 3.49;
}

#endif