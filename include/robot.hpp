#ifndef JACOBIAN_HPP
#define JACOBIAN_HPP

#include <Eigen/Dense>
// #include "kinematics.hpp"
#include "se3.hpp"

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
    /**
     * @brief Current configuration of the joints
    */
    JointStateVector q;

    /**
     * @brief Construct a new Robot object
    */

    Robot(JointStateVector q);
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
    Eigen::Matrix<double, 6, 6> jacobian();

    /**
     * @brief Forward kinematics
     * @param q Joint state vector
     */
    SE3 forwardKinematics(JointStateVector &q);

   /**
    * @brief Inverse kinematics
    * @param T_des Desired end-effector transformation
    * @return Joint state vector
   */
    JointStateVector inverseKinematics(SE3 &T_des);

    /**
     * @brief Move the robot
     * 
     * @param dt Time step
     * @param v_ref Desired velocity
     * @param q_des Desired joint state vector 
     */
    void move(double dt, double v_des, JointStateVector q_des);
};

#endif