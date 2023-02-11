#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include "config.hpp"
#include <Eigen/Dense>
#include "se3.hpp"
#include <cmath>

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
    
    static constexpr double workingHeight =  0.4550;
    static constexpr double descentHeight =  0.70;

    Joints joints;

    /**
     * @brief Current configuration of the joints
    */
    //VEC6 q;
    VEC6 q_home;
    VEC3 pose;

    /**
     * @brief Current configuration of the gripper
    */
    //VEC3 q_gripper;

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
    void moveGripper(double d, int N, double dt);
    void move(VEC3 &pose);
    void descent(double h, double rotation, bool pick);
};

class Controller{
    static constexpr double q1min = 0;
    static constexpr double q1max = M_PI;
    static constexpr double q1avg = M_PI / 2.0;
    static constexpr double q2min = 0;
    static constexpr double q2max = M_PI;
    static constexpr double q2avg = M_PI / 2.0;
    static constexpr double q3min = - M_PI;
    static constexpr double q3max = 0;
    static constexpr double q3avg = - M_PI / 2.0;
    static constexpr double q4min = - M_PI / 2.0;
    static constexpr double q4max = 0;
    static constexpr double q4avg = M_PI / 2.0;
    static constexpr double q5min = - M_PI / 2.0;
    static constexpr double q5max = 0;
    static constexpr double q5avg = M_PI / 2.0;
    static constexpr double q6min = - M_PI / 2.0;
    static constexpr double q6max = 0;
    static constexpr double q6avg = M_PI / 2.0;
public:
    static constexpr double dt = 0.001;
    static constexpr double T = 2;
    static void redundantController(Robot &r, VEC3 &x_f);
    static VEC6 computeQ0dot(VEC6 q);
    static  VEC6 computeQdot(MAT6 &Jac, VEC6 q, VEC3 xe, VEC3 xd, VEC3 vd);

    /**
     * @brief 
     * 
     * @param r robot
     * @param rpy_f roll, pitch, yaw of the final position
     */
    static void redundantControllerRotation(Robot &r, VEC3 &rpy_f);
    
    /**
     * @brief Move the robot
     * 
     * @param dt Time step
     * @param v_ref Desired velocity
     * @param q_des Desired joint state vector 
     */
    static void velocityController(Robot &r, double dt, double v_des, VEC6 q_des);

};


#endif
