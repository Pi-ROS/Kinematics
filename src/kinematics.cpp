#include "../include/kinematics.hpp"

/*
* @brief: Inverse kinematics solver
* @param: robot: robot model
* @param: q: initial joint state
* @param: oMdes: desired end-effector pose
* @return: q: joint state that achieves desired end-effector pose
*/

JointStateVector inverseKinematicsSolver(Model &robot, JointStateVector &q, SE3 &oMdes)
{
    Data data(robot);

    const int JOINT_ID = 6; // end-effector
    const double eps = 1e-4;
    const int IT_MAX = 1000;
    const double DT = 1e-1;
    const double damp = 1e-6;

    pinocchio::Data::Matrix6x J(6, robot.nv);
    J.setZero();

    bool success = false;
    JointStateVector err;
    Eigen::VectorXd v(robot.nv);

    // Main loop
    for (int i = 0;; i++)
    {
        // Compute the pose of the end effector given the
        // current joint configuration q
        pinocchio::forwardKinematics(robot, data, q);

        const pinocchio::SE3 dMi = oMdes.actInv(data.oMi[JOINT_ID]);
        // Compute the difference between the current end effector pose
        // and the desired end effector pose
        err = pinocchio::log6(dMi).toVector();
        if (err.norm() < eps)
        {
            // The error is small enough, we can stop here
            success = true;
            break;
        }
        if (i >= IT_MAX)
        {
            // The maximum number of iterations has been reached
            success = false;
            break;
        }

        // Update the joint configuration
        pinocchio::computeJointJacobian(robot, data, q, JOINT_ID, J);
        pinocchio::Data::Matrix6 JJt;
        JJt.noalias() = J * J.transpose();
        JJt.diagonal().array() += damp;
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(robot, q, v * DT);
        if (!(i % 10))
            std::cout << i << ": error = " << err.transpose() << std::endl;
    }

    if (success)
        std::cout << "Convergence achieved!" << std::endl;
    else
        std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
    
    std::cout << "\nfinal error: " << err.transpose() << std::endl;
    return q;
}

Model getRobotModel(std::string &path) {
    Model model;
    pinocchio::urdf::buildModel(path, model);
    return model;
}