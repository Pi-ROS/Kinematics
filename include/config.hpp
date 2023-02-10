#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>

#define SIMULATION true
#define USE_GRIPPER true
#define SOFT_GRIPPER true


static Eigen::Matrix<double, 6, 1> Q_HOME;

inline void initConfig() {
    Q_HOME << -0.32, -0.78, -2.56,-1.63, -1.57, 3.49;
}

#endif