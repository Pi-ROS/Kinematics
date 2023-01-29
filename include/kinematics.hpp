#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include "ros/ros.h"
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>

using namespace pinocchio;

typedef Eigen::Matrix<double, 6, 1> JointStateVector;

JointStateVector inverseKinematicsSolver(Model &robot, JointStateVector &q, SE3 &oMdes);
Model getRobotModel(std::string &path);

#endif