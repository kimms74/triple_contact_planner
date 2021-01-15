#pragma once

#include "triple_contact_planner/robot_dynamics/robot_dynamics_model.h"

namespace contact_planner
{

class DexterousRobotModel : public RobotDynamicsModel
{
public:
  bool isReachable(Eigen::Vector3d position) override
  {
    // ROS_INFO("%lf %lf %lf", position[0], position[1], position[2]);

    return (position.norm() < 100);
  }
  bool isPossibleContact(Eigen::Affine3d transform) override { return true; }

  Eigen::Vector3d getMaximumForce();
  Eigen::Vector3d getMaximumMoment();

  // for grasp contact
  Eigen::Matrix<double, 2, 6> getForceLimitTop() override
  {
    Eigen::Matrix<double, 2, 6> limit_matrix;
    for (int i = 0; i < 3; i++)
    {
      limit_matrix(0, i) = -30;
      limit_matrix(1, i) = -30;
    }

    for (int i = 3; i < 6; i++)
    {
      limit_matrix(0, i) = -2.24;
      limit_matrix(1, i) = -2.24;
    }

    return limit_matrix;
  }

    Eigen::Matrix<double, 2, 6> getForceLimitBottom() override
  {
    Eigen::Matrix<double, 2, 6> limit_matrix;
    for (int i = 0; i < 3; i++)
    {
      // limit_matrix(0, i) = -27.05;
      // limit_matrix(1, i) = -27.05;
      limit_matrix(0, i) = -30;
      limit_matrix(1, i) = -30;
    }

    limit_matrix.block<1,1>(0,2) << 0;

    for (int i = 3; i < 6; i++)
    {
      // limit_matrix(0, i) = 1;
      // limit_matrix(1, i) = -10;
      limit_matrix(0, i) = -10;
      limit_matrix(1, i) = -10;
    }

    return limit_matrix;
  }
};

}
