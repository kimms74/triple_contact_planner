#include "triple_contact_planner/solver/qp_solver.h"

namespace contact_planner
{

  QpSolver::QpSolver()
  {
    // com_ << -0.40664, 0.12478, 0.18233;
    com_ << 0.25793, 0.39568, -0.18497;
    com_T_.linear().setIdentity();
    com_T_.translation() = -com_;
    
  }

  Eigen::Quaterniond QpSolver::make_quaternion(const Eigen::VectorXd &tf)
  {
    Eigen::Quaterniond quat;
    quat.coeffs() = tf.tail<4>();
    return quat;
  }

  Eigen::Affine3d QpSolver::get_tf(int i)
  {
    return tf_.at(i);
  }

  Eigen::MatrixXd QpSolver::get_result_force(int i)
  {
    return contact_nodes_.at(i)->getContactForceTorque();
  }

  bool QpSolver::qp_solve(const Eigen::VectorXd &tf1, const Eigen::VectorXd &tf2, const Eigen::VectorXd &tf3, const Eigen::VectorXd &object_rotation)
  {
    Eigen::Quaterniond rotation_quaternion = make_quaternion(object_rotation);
    object_rotation_ = rotation_quaternion.matrix();

    std::vector<Eigen::VectorXd> tf(3);
    tf.at(0)= tf1;
    tf.at(1)= tf2;
    tf.at(2)= tf3; 

    tf_.resize(3);

    for(int i=0; i < 3; i++)
    {
      tf_.at(i).setIdentity();
      tf_.at(i).linear() = make_quaternion(tf.at(i)).matrix();
      tf_.at(i).translation()=tf.at(i).head<3>();
    }

    op_.setRobot(robot_model_);
    op_.setObjectInitOri();
    op_.setObjectRotation(object_rotation_);
    op_.setTopBottomNumber(tf);

    contact_nodes_.resize(3);

    for (auto &p : contact_nodes_)
    p = make_shared<Contact>();

    double chair_mass = 3.75;
    double side_left_mass = 1;
    double side_right_with_long_short_middle_mass = 2.75;

    model_->setMass(chair_mass);
    op_.setTopBottomMass(side_left_mass, side_right_with_long_short_middle_mass);

    for (int i=0;i < 3; i++)
    {
      contact_nodes_.at(i)->setTransform(com_T_ * tf_.at(i));
    }

    model_->setContactRobot(contact_nodes_);
    op_.setModel(model_);

    if (op_.solve())
    {
      return true;
    }
    return false;
  }

  bool QpSolver::qp_solve_one_body(const Eigen::VectorXd &tf1, const Eigen::VectorXd &tf2, const Eigen::VectorXd &tf3, const Eigen::VectorXd &object_rotation)
  {
    Eigen::Quaterniond rotation_quaternion = make_quaternion(object_rotation);
    object_rotation_ = rotation_quaternion.matrix();
    
    std::vector<Eigen::VectorXd> tf(3);
    tf.at(0)= tf1;
    tf.at(1)= tf2;
    tf.at(2)= tf3; 

    tf_.resize(3);
    
    for(int i=0; i < 3; i++)
    {
      tf_.at(i).setIdentity();
      tf_.at(i).linear() = make_quaternion(tf.at(i)).matrix();
      tf_.at(i).translation()=tf.at(i).head<3>();
    }
    
    op_.setRobot(robot_model_);
    op_.setObjectInitOri();
    op_.setObjectRotation(object_rotation_);
    op_.setTopBottomNumber(tf);

    contact_nodes_.resize(3);

    for (auto &p : contact_nodes_)
    p = make_shared<Contact>();

    double chair_mass = 3.75;

    model_->setMass(chair_mass);

    for (int i=0;i < 3; i++)
    {
      contact_nodes_.at(i)->setTransform(com_T_ * tf_.at(i));
    }

    model_->setContactRobot(contact_nodes_);
    op_.setModel(model_);
    
    if (op_.solve_one_body())
    {
      return true;
    }
    return false;
  }
}