#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "config.hpp"
#include <Eigen/Dense>
#include "utils.hpp"
#include <cmath>
#include <string>

#define WORKING_HEIGHT 0.4550

/* The descent height */
#if SIMULATION

    #if SOFT_GRIPPER
    #define DESCENT_HEIGHT 0.745
    #else
    #define DESCENT_HEIGHT 0.71
    #endif

#else 

    // This parameters should be tuned on the real robot
    #if SOFT_GRIPPER
    #define DESCENT_HEIGHT 0.735
    #else
    #define DESCENT_HEIGHT 0.69
    #endif

#endif

class Joints{
    
public:
    double shoulder_lift;
    double shoulder_pan;
    double elbow;
    double wrist_1;
    double wrist_2;
    double wrist_3;

    double hand_1;
    double hand_2;
    double hand_3;

    Joints() = default;
    Joints(VEC6 q, VEC3 gripper);
    VEC6 q();
    VEC3 q_gripper();
    void update();
    void update(VEC6 q);
    void update(VEC3 gripper);
    void update(VEC6 q, VEC3 gripper);
};


class Robot
{

private:
    /**
     * @brief UR5 DH parameters
    */
    static constexpr double d1 = 0.163;
    static constexpr double a2 = -0.42500;
    static constexpr double a3 = -0.39225;
    static constexpr double d4 = 0.134;
    static constexpr double d5 = 0.100;
    static constexpr double d6 = 0.100;

public:
    Joints joints;
    VEC6 q_home;
    VEC3 pose;

    /**
     * @brief Construct a new Robot object
    */
    Robot() = default;
    Robot(VEC6 q);
    /**
     * @brief Transformation matrix from frame 0 to frame 1
    */
    SE3 T01(double theta1);

    /**
     * @brief Transformation matrix from frame 1 to frame 2
    */
    SE3 T12(double theta2);
    
    /**
     * @brief Transformation matrix from frame 2 to frame 3
    */
    SE3 T23(double theta3);
    
    /**
     * @brief Transformation matrix from frame 3 to frame 4
    */
    SE3 T34(double theta4);
    
    /**
     * @brief Transformation matrix from frame 4 to frame 5
    */
    SE3 T45(double theta5);
    
    /**
     * @brief Transformation matrix from frame 5 to frame 6
    */
    SE3 T56(double theta6);

    /**
     * @brief Jacobian of the end-effector transformation with respect to
     * the current joints coordinates.
    */
    MAT6 jacobian(VEC6 q);

    /**
     * @brief Forward kinematics
     * @param q Joint state vector
     */
    SE3 forwardKinematics(VEC6 &q);

   /**
    * @brief Inverse kinematics
    * @param T_des Desired end-effector transformation
    * @return Joint state vector
   */
    VEC6 inverseKinematics(SE3 &T_des);
    /**
     * @brief Moves the gripper to the desired opening diameter.
     * @param d the desired opening diameter
     * @param N number of steps to complete the movement
     * @param dT duration of a single step
    */
    void move(SE3 &T_des);
    void descent(SE3 &T_des, bool pick, ros::ServiceClient &gripperClient);
    void moveGripper(ros::ServiceClient &gripperClient, double d, int N, double dt);
};

#endif
