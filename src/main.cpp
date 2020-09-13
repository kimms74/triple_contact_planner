#include <iostream>
#include <fstream>
#include "triple_contact_planner/robot_dynamics/dexterous_robot_model.h"
#include "triple_contact_planner/solver/contact_optimization.h"
#include "triple_contact_planner/cont_reader.h"
#include "triple_contact_planner/contact_model/stefan_model.h"
#include <ros/ros.h>
#include <algorithm>
#include "triple_contact_planner/permutation.h"

#include <ros/package.h>
using namespace std;
using namespace suhan_contact_planner;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "suhan_contact_planner");
  ros::NodeHandle nh;
  std::string path = ros::package::getPath("triple_contact_planner");

  ContinuousGraspCandid grp_top;
  grp_top.loadConfig(path + "/top.yaml");

  ContinuousGraspCandid grp_bottom;
  grp_bottom.loadConfig(path + "/bottom.yaml");

  ContactOptimization op;
  RobotDynamicsModelPtr robot_model = make_shared<DexterousRobotModel>();
  op.setRobot(robot_model);

  Eigen::Vector3d com;
  com << -0.40664, 0.12478, 0.18233;
  Eigen::Affine3d com_T;
  com_T.setIdentity();
  com_T.translation() = -com;

  std::vector<ContactPtr> contact_nodes_;
  contact_nodes_.resize(3);

  for (auto &p : contact_nodes_)
    p = make_shared<Contact>();

  std::vector<int> links;
  for (int i = 0; i < grp_bottom.candids_set.size(); i++)
    links.push_back(i);

  ContactModelPtr model = make_shared<StefanModel>("hi");
  model->setMass(3.75);

  vector<vector<int>> all_combination = get_combinations_from_vector<int>(links, 2);
  vector<vector<int>> tot_combination;
  for (auto combination : all_combination)
  {
    for (int i = 0; i < 7; i++)
    {
      combination.push_back(i);
      tot_combination.push_back(combination);
      combination.pop_back();
    }
  }

  vector<vector<double>> ratio_combination;
  vector<double> test;
  DFS(ratio_combination, test);

  std::ofstream file;
  file.open(path + "/start_static_eq_grp.yaml");

  Eigen::IOFormat CommaInitFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
  int i = 0;
  tuple<Eigen::Affine3d, std::string, double> result[3];
  for (auto combination : tot_combination)
  {
    for (auto ratio_combi : ratio_combination)
    {
      for (int j = 0; j < grp_bottom.candids_set[combination[0]].size(); j++)
      {
        result[0] = grp_bottom.getGrasp(combination[0], j, ratio_combi[0]);
        contact_nodes_.at(0)->setTransform(com_T * get<0>(result[0]));

        for (int k = 0; k < grp_bottom.candids_set[combination[1]].size(); k++)
        {
          result[1] = grp_bottom.getGrasp(combination[1], k, ratio_combi[1]);
          contact_nodes_.at(1)->setTransform(com_T * get<0>(result[1]));

          for (int m = 0; m < grp_top.candids_set[combination[2]].size(); m++)
          {
            result[2] = grp_top.getGrasp(combination[2], m, ratio_combi[2]);
            contact_nodes_.at(2)->setTransform(com_T * get<0>(result[2]));

            model->setContactRobot(contact_nodes_);
            op.setModel(model);
            if (op.solve())
            {
              bool write = true;
              for (int i = 0; i < 3; i++)
              {
                if (get<2>(result[i]) < 0.15 && ratio_combi[i] != 0.5)
                {
                  write = false;
                  break;
                }
              }
              if (write)
              {
                file << "- part num : [" << combination[0] + 1 << ", " << combination[1] + 1 << ", " << combination[2] + 1 << "]" << endl
                     << "  index : [" << j << ", " << k << ", " << m << "]" << endl
                     << "  ratio : [" << ratio_combi[0] << ", " << ratio_combi[1] << ", " << ratio_combi[2] << "]" << endl
                     << "  ori : [" << get<1>(result[0]) << ", " << get<1>(result[1]) << ", " << get<1>(result[2]) << "]" << endl;

                // offset
                // for (int t = 0; t < 3; t++)
                //   get<0>(result[t]).translation() += get<0>(result[t]).linear() * Eigen::Vector3d(0, 0, -0.103);

                Eigen::VectorXd t1(7), t2(7), t3(7);
                t1 << get<0>(result[0]).translation()+ get<0>(result[0]).linear()*Eigen::Vector3d(0, 0, -0.103), 
                      Eigen::Quaterniond(get<0>(result[0]).linear()).coeffs();
                t2 << get<0>(result[1]).translation()+ get<0>(result[1]).linear()*Eigen::Vector3d(0, 0, -0.103),
                      Eigen::Quaterniond(get<0>(result[1]).linear()).coeffs();
                t3 << get<0>(result[2]).translation()+ get<0>(result[2]).linear()*Eigen::Vector3d(0, 0, -0.103),
                      Eigen::Quaterniond(get<0>(result[2]).linear()).coeffs();
                // t3 << Eigen::Map<Eigen::VectorXd>( ( get<0>(result[2]).translation()+ get<0>(result[2]).linear()*Eigen::Vector3d(0, 0, -0.103)).data(), 3)  , 
                //                                   Eigen::Quaterniond(get<0>(result[2]).linear()).coeffs();

                // Eigen::VectorXd t1(Eigen::Map<Eigen::VectorXd>(get<0>(result[0]).matrix().data(), 16));
                // Eigen::VectorXd t2(Eigen::Map<Eigen::VectorXd>(get<0>(result[1]).matrix().data(), 16));
                // Eigen::VectorXd t3(Eigen::Map<Eigen::VectorXd>(get<0>(result[2]).matrix().data(), 16));
                file << "  transform1 : " << t1.transpose().format(CommaInitFmt) << endl
                     << "  transform2 : " << t2.transpose().format(CommaInitFmt) << endl
                     << "  transform3 : " << t3.transpose().format(CommaInitFmt) << endl;
                //  << "  transform1 : " << endl
                //  << get<0>(result[0]).matrix() << endl
                //  << "  transform2 : " << endl
                //  << get<0>(result[1]).matrix() << endl
                //  << "  transform3 : " << endl
                //  << get<0>(result[2]).matrix() << endl;
                // file << "      force 1 : " << contact_nodes_.at(0)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
                //      << "      force 2 : " << contact_nodes_.at(1)->getContactForceTorque().transpose().format(CommaInitFmt) << endl
                //      << "      force 3: " << contact_nodes_.at(2)->getContactForceTorque().transpose().format(CommaInitFmt) << endl;
                i++;
              }

            }
          }
        }
      }
    }
  }
  std::cout << i << std::endl;
  file.close();
  return 0;
}
