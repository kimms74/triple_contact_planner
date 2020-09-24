#include <iostream>
#include <fstream>
#include "triple_contact_planner/robot_dynamics/dexterous_robot_model.h"
#include "triple_contact_planner/solver/contact_optimization.h"
#include "triple_contact_planner/cont_reader.h"
#include "triple_contact_planner/contact_model/stefan_model.h"
#include <ros/ros.h>
#include <algorithm>
#include "triple_contact_planner/permutation.h"
#include "triple_contact_planner/solver/qp_solver.h"

#include <ros/package.h>
using namespace std;
using namespace suhan_contact_planner;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "suhan_contact_planner");
  ros::NodeHandle nh;
  std::string path = ros::package::getPath("triple_contact_planner");

  Eigen::VectorXd tf1(7), tf2(7), tf3(7);
  Eigen::VectorXd object_rotation;

  QpSolver qp;

  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

  Eigen::Matrix3d rotation(3,3);
//   rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  rotation << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  object_rotation = Eigen::Quaterniond(rotation).coeffs();
  
  tf1 << -0.268485, 0.397779, 0.0213787, 0.999296, 3.40748e-08, -1.27928e-09, -0.0375168;
  tf2 << -0.411417, 0.212699, 0.0213731, -0.0114453, 0.00906421, 0.720166, 0.693648;
  tf3 << -0.38889, 0.0150407, 0.174111, 0.706609, 0.0265277, -0.706609, -0.0265276;
  

  if(qp.qp_solve(tf1,tf2,tf3,object_rotation))
  { 
    std::ofstream file;
    file.open(path + "/qp_solve_test.yaml");

    Eigen::VectorXd t1(7), t2(7), t3(7);
//     t1 << qp.get_tf(0).translation()+ qp.get_tf(0).linear()*Eigen::Vector3d(0, 0, -0.103), 
//           Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
//     t2 << qp.get_tf(1).translation()+ qp.get_tf(1).linear()*Eigen::Vector3d(0, 0, -0.103), 
//           Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
//     t3 << qp.get_tf(2).translation()+ qp.get_tf(2).linear()*Eigen::Vector3d(0, 0, -0.103), 
//           Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();

      t1 << qp.get_tf(0).translation(), Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
      t2 << qp.get_tf(1).translation(), Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
      t3 << qp.get_tf(2).translation(), Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();

    file << "  transform1 : " << t1.transpose().format(CommaInitFmt) << endl
        << "  transform2 : " << t2.transpose().format(CommaInitFmt) << endl
        << "  transform3 : " << t3.transpose().format(CommaInitFmt) << endl;

    file << "  force 1 : " << qp.get_result_force(0)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
          << "  force 2 : " << qp.get_result_force(1)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
          << "  force 3: " << qp.get_result_force(2)->getContactForceTorque().transpose().format(CommaInitFmt) << endl;
  }
  return 0;
      
}
