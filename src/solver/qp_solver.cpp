#include "triple_contact_planner/solver/qp_solver.h"

namespace suhan_contact_planner
{
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

  ContactPtr QpSolver::get_result_force(int i)
  {
    return contact_nodes_.at(i);
  }

  bool QpSolver::qp_solve(const Eigen::VectorXd &tf1, const Eigen::VectorXd &tf2, const Eigen::VectorXd &tf3, const Eigen::VectorXd &object_rotation)
  {
    Eigen::MatrixXd object_rot(3,3);
    object_rotation_ = object_rotation.matrix();

    std::vector<Eigen::VectorXd> tf(3);
    tf.at(0)= tf1;
    tf.at(1)= tf2;
    tf.at(2)= tf3; 

    tf_.resize(3);
    
    Eigen::Quaterniond quat;
    for(int i=0; i < 3; i++)
    {
      tf_.at(i).setIdentity();
      tf_.at(i).linear() = make_quaternion(tf.at(i)).matrix();
      tf_.at(i).translation()=tf.at(i).head<3>();
    }
    
    op_.setRobot(robot_model_);
    
    op_.setObjectRotation(object_rotation_);

    com_ << -0.40664, 0.12478, 0.18233;
    com_T_.linear().setIdentity();
    com_T_.translation() = -com_;

    contact_nodes_.resize(3);

    for (auto &p : contact_nodes_)
    p = make_shared<Contact>();

    model_->setMass(3.75);

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
}