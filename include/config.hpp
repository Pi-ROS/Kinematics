#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>
#include "ros.hpp"
#define SIMULATION false
#define USE_GRIPPER false
#define SOFT_GRIPPER false
#define TASK0 true
#define DT 0.001

static Eigen::Matrix<double, 6, 1> Q_HOME;

inline void initConfig() {
    Q_HOME << -0.32, -0.78, -2.56,-1.63, -1.57, 3.49;
}

#endif