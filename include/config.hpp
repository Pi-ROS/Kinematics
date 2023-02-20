#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <Eigen/Dense>
#include "ros.hpp"

#define SIMULATION false
#define USE_GRIPPER true
#define SOFT_GRIPPER false // True: 2-fingers gripper, False: 3-fingers gripper

#if SOFT_GRIPPER
    #define GRIPPER_OFFSET 0.13
#else
    #define GRIPPER_OFFSET 0.13 // ** TO BE MESURED IN THE LABORATORY **
#endif

#define TASK0 false
#define TASK_SELECTION 1
#define DT 0.001


#if SIMULATION

    #define VELOCITY 2

    #if TASK_SELECTION == 3 || TASK_SELECTION == 4
        // A virtual link is used to attach the brick to the gripper
        #define GRIPPER_OPENING 180
    #else 
        #define GRIPPER_OPENING 15
    #endif

#else
    #define VELOCITY 0.2
    #define GRIPPER_OPENING 30
#endif

#endif
