#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>
#include "ros.hpp"

#define SIMULATION true
#define USE_GRIPPER true
#define SOFT_GRIPPER false // True: 2-fingers gripper, False: 3-fingers gripper
#define TASK0 false
#define TASK_SELECTION 1


#if SIMULATION
#define VELOCITY 2
#define GRIPPER_OPENING 15
#else
#define VELOCITY 0.2
#define GRIPPER_OPENING 55
#endif
#define DT 0.001

#endif