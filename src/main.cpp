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
using namespace contact_planner;

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "contact_planner");
  ros::NodeHandle nh;
  std::string path = ros::package::getPath("triple_contact_planner");

  Eigen::Matrix<double,7,1> tf1, tf2, tf3;
  Eigen::VectorXd object_rotation;

  QpSolver qp;


  Eigen::Matrix3d rotation(3,3);
  // rotation << 1, 0, 0, 0, 0, -1, 0, 1, 0;
  // rotation << 1, 0, 0, 0, -1, 0, 0, 0, -1;
  // rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
  // rotation << 1, 0, 0, 0, 0.0871557, -0.9961947, 0, 0.9961947, 0.0871557; //x:85 degrees
  rotation << 1, 0, 0, 0, -0.8660254, -0.5, 0, 0.5, -0.8660254; //x: 95 degrees
  // rotation << 1, 0, 0, 0, -0.1736482, -0.9848077, 0, 0.9848077, -0.1736482;  //x: 150 degrees

  object_rotation = Eigen::Quaterniond(rotation).coeffs();
  YAML::Node yamlnode;

  //bottom2_top1일 때와 top2_bottom1일 때 파일을 바꿔줘야한다
  yamlnode = YAML::LoadFile(path + "/yaml/permutation/bottom2_top1_permutation.yaml");
  // yamlnode = YAML::LoadFile(path + "/yaml/permutation/bottom1_top2_permutation.yaml");

  Eigen::VectorXd t1(7), t2(7), t3(7);
  
  
  std::ofstream file;
  file.open(path + "/qp_solve_test.yaml");

  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

  for(int i=0; i < yamlnode.size();i++)
    {
      tf1 = Eigen::Matrix<double,7,1>::Map(yamlnode[i]["tf1"].as<std::vector<double>>().data());
      tf2 = Eigen::Matrix<double,7,1>::Map(yamlnode[i]["tf2"].as<std::vector<double>>().data());
      tf3 = Eigen::Matrix<double,7,1>::Map(yamlnode[i]["tf3"].as<std::vector<double>>().data());
    

      if(qp.qp_solve(tf1,tf2,tf3,object_rotation))
      { 


        // t1 << qp.get_tf(0).translation()+ qp.get_tf(0).linear()*Eigen::Vector3d(0, 0, -0.103), 
        //       Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
        // t2 << qp.get_tf(1).translation()+ qp.get_tf(1).linear()*Eigen::Vector3d(0, 0, -0.103), 
        //       Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
        // t3 << qp.get_tf(2).translation()+ qp.get_tf(2).linear()*Eigen::Vector3d(0, 0, -0.103), 
        //       Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();

          t1 << qp.get_tf(0).translation(), Eigen::Quaterniond(qp.get_tf(0).linear()).coeffs();
          t2 << qp.get_tf(1).translation(), Eigen::Quaterniond(qp.get_tf(1).linear()).coeffs();
          t3 << qp.get_tf(2).translation(), Eigen::Quaterniond(qp.get_tf(2).linear()).coeffs();

        file << "  - transform1 : " << t1.transpose().format(CommaInitFmt) << endl
            << "    transform2 : " << t2.transpose().format(CommaInitFmt) << endl
            << "    transform3 : " << t3.transpose().format(CommaInitFmt) << endl;

        file << "    force 1 : " << qp.get_result_force(0).transpose().format(CommaInitFmt) << endl
              << "    force 2 : " << qp.get_result_force(1).transpose().format(CommaInitFmt) << endl
              << "    force 3: " << qp.get_result_force(2).transpose().format(CommaInitFmt) << endl;
      }
    }
    return 0;
      
}
