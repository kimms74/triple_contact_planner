#pragma once
#include "triple_contact_planner/robot_dynamics/dexterous_robot_model.h"
#include "triple_contact_planner/solver/contact_optimization.h"
#include "triple_contact_planner/cont_reader.h"
#include "triple_contact_planner/contact_model/stefan_model.h"
#include <iostream>
#include <fstream>
#include <algorithm>

namespace suhan_contact_planner
{

class QpSolver
{
private:
  ContactOptimization op_;
  RobotDynamicsModelPtr robot_model_ = make_shared<DexterousRobotModel>();
  ContinuousGraspCandid grp_top_;
  ContinuousGraspCandid grp_bottom_;
  Eigen::MatrixXd object_rotation_;
  Eigen::Vector3d com_;
  Eigen::Affine3d com_T_;
  std::vector<Eigen::Affine3d> tf_;
  std::vector<ContactPtr> contact_nodes_;
  ContactModelPtr model_ = make_shared<StefanModel>("hi");

public:
  Eigen::Quaterniond make_quaternion(const Eigen::VectorXd &tf);
  bool qp_solve(const Eigen::VectorXd &tf1, const Eigen::VectorXd &tf2, const Eigen::VectorXd &tf3, const Eigen::VectorXd &object_rotation);
  Eigen::Affine3d get_tf(int i);
  ContactPtr get_result_force(int i);
};

}