#pragma once

#include <limits>

#include <triple_contact_planner/cont_reader.h>
#include <triple_contact_planner/contact_model/contact_model.h>
#include <triple_contact_planner/robot_dynamics/robot_dynamics_model.h>
#include <triple_contact_planner/solver/constraint/constraint_equality.h>
#include <triple_contact_planner/solver/constraint/constraint_inequality.h>
#include <triple_contact_planner/solver/contact_optimization_solver.h>

namespace contact_planner
{

class ContactOptimization
{
public:
  void setModel(ContactModelPtr model);
  void setRobot(RobotDynamicsModelPtr robot);
  void setObjectInitOri();
  void setObjectRotation(Eigen::MatrixXd object_rotation);
  bool solve();
  bool solve_one_body();
  void setBottom(const double &contact_number, const double &contact_bottom_number, const std::vector<ContactPtr> &contacts, const Eigen::VectorXd &normal_force, ContactOptimizationSolver &solver_bottom);
  void setTop(const double &contact_number,const double &contact_bottom_number, const double &contact_top_number, const std::vector<ContactPtr> &contacts, ContactOptimizationSolver &solver_top);
  void setConstraints(const double &contact_number,const double &contact_bottom_number, const double &contact_top_number, const std::vector<ContactPtr> &contacts, ContactOptimizationSolver &solver);
  void setTopBottomNumber(const std::vector<Eigen::VectorXd> &tf);
  void setContactNumberZero();
  void setTopBottomMass(double &side_left_mass, double &side_right_with_long_short_middle_mass);
  size_t getBottomNumber();
  size_t getTopNumber();
  double getTopMass();
  double getBottomMass();

private:
  ContactModelPtr model_;
  RobotDynamicsModelPtr robot_;
  Eigen::MatrixXd object_init_ori_;
  Eigen::MatrixXd object_rotation_;
  Eigen::Vector3d normal_vector_;
  size_t contact_bottom_number_;
  size_t contact_top_number_;
  double top_mass_;
  double bottom_mass_;
  vector<int> top_bottom_order_;
};

}
