#ifndef CONTROLLERS_HPP
#define CONTROLLERS_HPP

#include <cmath>
#include <Eigen/Dense>
#include "config.hpp"
#include "robot.hpp"

/**
 * @namespace vector_field_controller
 * @brief Namespace for a vector field based controller implementation.
 */

namespace vector_field_controller {
    /**
     * @brief Scaling factor for the translational velocity.
     */
    #if SIMULATION
    constexpr double Lambda = 5;
    #else
    constexpr double Lambda = 0.75
    #endif

    /**
     * @brief The maximum allowed translational velocity.
     */
    constexpr double MaxVel = VELOCITY;

    /**
     * @brief The maximum allowed rotational velocity.
     */
    #if SIMULATION
    constexpr double MaxRotVel = M_PI/2;
    #else
    constexpr double MaxRotVel = M_PI/4;
    #endif

    /**
     * @brief The range of the vector field, expressed in meters as a radial distance
     * from the z axis of the base frame.
     */
    constexpr double r0 = 0.2;

    /**
     * @brief Scaling term.
     */
    constexpr double N0 = 200;

    /**
     * @brief An error threshold for the inverse kinematics.
     */
    constexpr double ErrThresh = 0.02;
    
    /**
     * @brief Computes the vector field.
     * @param p_curr The current position of the end effector.
     * @param p_f The desired position of the end effector.
     */
    VEC3 vectorField(VEC3 p_curr, VEC3 p_f);

    /**
     * @brief Computes the desired rotational velocity.
     * @param e The rotation error, expressed through the angle-axis representation.
     * @param iter The current iteration of the controller.
     */
    double scalarRotVelocity(VEC3 e, int iter);

    /**
     * @brief Computes the desired translational velocity.
     * @param e The translation error.
     * @param iter The current iteration of the controller.
     */
    double scalarVelocity(VEC3 e, int iter);

    /**
     * @brief Computes the desired rotational velocity that will be
     * used in the inverse kinematics solver.
     * @param e The current rotation error
     * @param iter The current iteration of the controller
     */
    VEC3 rotationalVelocity(VEC3 e, int iter);

    /**
     * @brief Computes the actual translational velocity that will be
     * used in the inverse kinematics solver.
     * @param e The current translation error
     * @param p_curr The current position of the end effector
     * @param p_f The desired position of the end effector
     * @param iter The current iteration of the controller
     */
    VEC3 velocity(VEC3 e, VEC3 p_curr, VEC3 p_f, int iter);

    /**
     * @brief Computes rotational velocity for each of the joints.
     * @param r The robot.
     * @param q The current joint configuration.
     * @param e The current translation and rotation error.
     * @param p_f The desired position of the end effector.
     * @param iter The current iteration of the controller.
     */
    VEC6 qDot(Robot &r, VEC6 q, VEC6 e, VEC3 p_f, int iter);

    /**
     * @brief Numeric solver of the inverse kinematics problem, which makes
     * use of the vector field.
     * @param r The robot.
     * @param T_des the desired end effector pose.
     */
    void vectorFieldController(Robot &r, SE3 &T_des);
}

/**
 * @namespace vector_field_controller
 * @brief Namespace for a redundacy exploiting controller implementation.
 */

namespace redundant_controller {
    /**
     * @brief The joint limits.
     */
    constexpr double q1min = -2*M_PI;
    constexpr double q1max = 2*M_PI;
    constexpr double q1avg = 0;
    constexpr double q2min = -2*M_PI;
    constexpr double q2max = 2*M_PI;
    constexpr double q2avg = 0;
    constexpr double q3min = -2*M_PI;
    constexpr double q3max = 2*M_PI;
    constexpr double q3avg = 0;
    constexpr double q4min = -2*M_PI;
    constexpr double q4max = 2*M_PI;
    constexpr double q4avg = 0;
    constexpr double q5min = -2*M_PI;
    constexpr double q5max = 2*M_PI;
    constexpr double q5avg = 0;
    constexpr double q6min = -2*M_PI;
    constexpr double q6max = 2*M_PI;
    constexpr double q6avg = 0;

    /**
     * @brief The desired time for the motion of the robot.
     */
    constexpr double T = 2;

    /**
     * @brief Computes a joint velocity biased towards a 
     * median joint configuration.
     * @param q The current joint configuration.
     */
    VEC6 computeQ0dot(VEC6 q);

    /**
     * @brief Computes rotational velocity for each of the joints.
     * @param Jac The robot jacobian.
     * @param q The current joint configuration.
     * @param xe The end effector position.
     * @param xd The desired end effector position.
     * @param vd The desired translational velocity.
     */
    VEC6 computeQdot(MAT6 &Jac, VEC6 q, VEC3 xe, VEC3 xd, VEC3 vd);

    /**
     * @brief Numeric solver of the inverse kinematics problem, which
     * exploits redundancy to achieve a desired end effector position,
     * but not orientation, while enforcing joint limits.
     * @param r The robot.
     * @param x_f The desired end effector position.
     */
    void redundantController(Robot &r, VEC3 &x_f);
};

/**
 * @brief An utility function to move the robot towards a desired joint configuration,
 * while publishing the intermediate joint configurations at a given frequency.
 * @param r The robot.
 * @param dt The reciprocal of the desired frequency.
 * @param v_des The maximum velocity.
 * @param q_f The target configuration.
 * @param ascent A flag that indicates wether the robot is moving upwards.
 */
void velocityController(Robot &r, double dt, double v_des, VEC6 q_f, bool ascent = false);

#endif