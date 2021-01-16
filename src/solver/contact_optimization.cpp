#include <triple_contact_planner/solver/contact_optimization.h>

namespace contact_planner
{

  Eigen::Matrix3d cross_skew(Eigen::Vector3d input)
  {
    Eigen::Matrix3d output;
    output.setZero();
    output(0, 1) = -input(2);
    output(1, 0) = input(2);
    output(0, 2) = input(1);
    output(2, 0) = -input(1);
    output(1, 2) = -input(0);
    output(2, 1) = input(0);
    return output;
  }

  void ContactOptimization::setModel(ContactModelPtr model)
  {
    model_ = model;
  }

  void ContactOptimization::setRobot(RobotDynamicsModelPtr robot)
  {
    robot_ = robot;
  }

  void ContactOptimization::setObjectInitOri()
  {
    Eigen::Quaterniond object_init_ori;
    object_init_ori.coeffs() << -0.9238795, 0.3826834, 0, 0;
      object_init_ori_ = object_init_ori.matrix();;
  }

  void ContactOptimization::setObjectRotation(Eigen::MatrixXd object_rotation)
  {
    object_rotation_ = object_rotation;
  }

  void ContactOptimization::setTopBottomNumber(const std::vector<Eigen::VectorXd> &tf)
  {
    setContactNumberZero();

    if (object_rotation_(2,2) > 0)
    {
      for(int i=0; i < 3; i++)
      {
        if (tf.at(i)(2) > -0.03)
        {
          contact_bottom_number_ += 1;
          //if tf.at(i) is top grasp point, push_back 0, if tf.at(i) is bottom grasp point, push_back 1
          top_bottom_order_.push_back(1);
        }
        else
        {
          contact_top_number_ += 1;
          top_bottom_order_.push_back(0);
        }
      }
    }
    else
    {
      for(int i=0; i < 3; i++)
      {
        if (tf.at(i)(2) > -0.03)
        {
          contact_top_number_ += 1;
          top_bottom_order_.push_back(0);
        }
        else
        {
          contact_bottom_number_ += 1;
          top_bottom_order_.push_back(1);
        }
      }
    }
  }
  void ContactOptimization::setContactNumberZero()
  {
    contact_bottom_number_ = 0;
    contact_top_number_ = 0;
    top_bottom_order_.clear();
  }
  size_t ContactOptimization::getBottomNumber()
  {
    return contact_bottom_number_;
  }  
  
  size_t ContactOptimization::getTopNumber()
  {
    return contact_top_number_;
  }

  void ContactOptimization::setTopBottomMass(double &side_left_mass, double &side_right_with_long_short_middle_mass)
  {
    if (object_rotation_(2,2) > 0)
    {
      top_mass_ = side_right_with_long_short_middle_mass;
      bottom_mass_ = side_left_mass;
    }
    else
    {
      top_mass_ = side_left_mass;
      bottom_mass_ = side_right_with_long_short_middle_mass;
    }
  }

  double ContactOptimization::getTopMass()
  {
    return top_mass_;
  }

  double ContactOptimization::getBottomMass()
  {
    return bottom_mass_;
  }

  void ContactOptimization::setTop(const double &contact_number,const double &contact_bottom_number, const double &contact_top_number, const std::vector<ContactPtr> &contacts, ContactOptimizationSolver &solver_top)
  {
    auto eq_constraint = std::make_shared<ConstraintEquality>();
    // eq_constraint.
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    // TODO: gravity
    A.setZero(6, (contact_top_number +1) * 6);
    b.setZero(6);
    // b.head<3>() = model_->getTransform().linear() *
    //               Eigen::Vector3d(0, 0, 9.8) * model_->getMass(); // TODO: Check this
    b.head<3>() =  Eigen::Vector3d(0, 0, 9.8) * getTopMass(); // TODO: Check this
    // TODO: Momentum + b(3~5)

    int j = 0;
    for (int i=0; i < contact_number; i++)
    {
      if (top_bottom_order_.at(i) == 0)
      {
        A.block<3, 3>(0, j * 6).setIdentity();
        A.block<3, 3>(3, j * 6) = cross_skew(object_init_ori_*object_rotation_*contacts[i]->getContactTransform().translation());
        A.block<3, 3>(3, j * 6 + 3).setIdentity();
        j++;
      }
    }

    A.block<3, 3>(0,(contact_top_number)*6).setIdentity();
    A.block<3, 3>(3, (contact_top_number)*6 + 3).setIdentity();

    eq_constraint->setA(A);
    eq_constraint->setEqualityCondition(b);

    solver_top.addConstraint(eq_constraint);

    // every contact
    auto ineq_constraint = std::make_shared<ConstraintInequality>();

    Eigen::MatrixXd C_all;
    Eigen::VectorXd d_all;
    d_all.resize(12*(contact_top_number +1));

    C_all.setZero(12 * (contact_top_number +1), 6 * (contact_top_number +1));

    Eigen::MatrixXd C_max[6];
    Eigen::VectorXd d_max[6];
    const auto &f_limit = robot_->getForceLimitTop();

    for (int i = 0; i < 6; i++)
    {
      C_max[i].setZero(2, 6);
      C_max[i](0, i) = 1;
      C_max[i](1, i) = -1;

      d_max[i].resize(2);
      d_max[i] = f_limit.col(i);
    }

    Eigen::MatrixXd C_i(12, 6), d_i(12, 1);

    for (int i = 0; i < 6; i++)
    {
      C_i.block<2, 6>(i * 2, 0) = C_max[i];
      d_i.block<2, 1>(i * 2, 0) = d_max[i];
    }

    Eigen::MatrixXd C_n(12, 6), d_n(12, 1);

    for (int i = 0; i < 6; i++)
    {
      C_n.block<2, 6>(i * 2, 0) = C_max[i];
      d_n.block<2, 1>(i * 2, 0) << 0, 0;
    }
    
    if (object_rotation_(2,2) > 0)
    {
      d_n.block<2,1>(4,0) << -15, 0;
    }
    else
    {
      d_n.block<2,1>(4,0) << 0, -15;
    }
    

    int k = 0;
    for (int i=0; i < contact_number; i++)
    {
      if (top_bottom_order_.at(i) == 0)
      {
        Eigen::Matrix<double, 6, 6> R_hat;
        Eigen::Matrix<double, 3, 3> R;
        R = contacts[i]->getContactTransform().linear().transpose()*(object_init_ori_*object_rotation_).transpose();
        R_hat.setZero();
        R_hat.block<3, 3>(0, 0) = R;
        R_hat.block<3, 3>(3, 3) = R;

        C_all.block<12, 6>(k * 12, k * 6) = C_i * R_hat;
        d_all.segment<12>(k * 12) = d_i;
        k++;
      }
    }

    Eigen::Matrix<double, 6, 6> R_hat;
    Eigen::Matrix<double, 3, 3> R;
    R = (object_init_ori_*object_rotation_).transpose();
    R_hat.setZero();
    R_hat.block<3, 3>(0, 0) = R;
    R_hat.block<3, 3>(3, 3) = R;

    C_all.block<12, 6>(contact_top_number * 12, contact_top_number * 6) = C_n * R_hat;
    d_all.segment<12>(contact_top_number * 12) = d_n;

    ineq_constraint->setA(C_all);
    ineq_constraint->setOnlyLowerBound(d_all);
    // ineq_constraint->printCondition();

    solver_top.addConstraint(ineq_constraint);
    solver_top.setContactNumber(contact_top_number+1);
  }

  void ContactOptimization::setBottom(const double &contact_number, const double &contact_bottom_number, const std::vector<ContactPtr> &contacts, const Eigen::VectorXd &normal_force, ContactOptimizationSolver &solver_bottom)
  {
    auto eq_constraint = std::make_shared<ConstraintEquality>();
    // eq_constraint.
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    // TODO: gravity
    A.setZero(6, contact_bottom_number * 6);
    b.setZero(6);
    // b.head<3>() = model_->getTransform().linear() *
    //               Eigen::Vector3d(0, 0, 9.8) * model_->getMass(); // TODO: Check this
    b.head<3>() = normal_force + Eigen::Vector3d(0, 0, 9.8) * getBottomMass();
    // TODO: Momentum + b(3~5)
    for (size_t i = 0; i < contact_bottom_number; i++)
    {
      A.block<3, 3>(0, i * 6).setIdentity();
      A.block<3, 3>(3, i * 6) = cross_skew(object_init_ori_*object_rotation_*contacts[i]->getContactTransform().translation());
      A.block<3, 3>(3, i * 6 + 3).setIdentity();
    }

    eq_constraint->setA(A);
    eq_constraint->setEqualityCondition(b);

    solver_bottom.addConstraint(eq_constraint);
    
    // every contact
    auto ineq_constraint = std::make_shared<ConstraintInequality>();

    Eigen::MatrixXd C_all;
    Eigen::VectorXd d_all;
    d_all.resize(12*contact_bottom_number);

    C_all.setZero(12 * contact_bottom_number, 6 * contact_bottom_number);

    Eigen::MatrixXd C_max[6];
    Eigen::VectorXd d_max[6];
    const auto &f_limit = robot_->getForceLimitBottom();

    for (int i = 0; i < 6; i++)
    {
      C_max[i].setZero(2, 6);
      C_max[i](0, i) = 1;
      C_max[i](1, i) = -1;

      d_max[i].resize(2);
      d_max[i] = f_limit.col(i);
    }

    Eigen::MatrixXd C_i(12, 6), d_i(12, 1);

    for (int i = 0; i < 6; i++)
    {
      C_i.block<2, 6>(i * 2, 0) = C_max[i];
      d_i.block<2, 1>(i * 2, 0) = d_max[i];
    }

    int k = 0;
    for (int i=0; i < contact_number; i++)
    {
      if (top_bottom_order_.at(i) == 1)
      {
        Eigen::Matrix<double, 6, 6> R_hat;
        Eigen::Matrix<double, 3, 3> R;
        R = contacts[i]->getContactTransform().linear().transpose()*(object_init_ori_*object_rotation_).transpose();
        R_hat.setZero();
        R_hat.block<3, 3>(0, 0) = R;
        R_hat.block<3, 3>(3, 3) = R;
        
        C_all.block<12, 6>(k * 12, k * 6) = C_i * R_hat;
        d_all.segment<12>(k * 12) = d_i;
        k++;
      }
    }

    ineq_constraint->setA(C_all);
    ineq_constraint->setOnlyLowerBound(d_all);
    // ineq_constraint->printCondition();

    solver_bottom.addConstraint(ineq_constraint);

    solver_bottom.setContactNumber(contact_bottom_number);
  }

  void ContactOptimization::setConstraints(const double &contact_number,const double &contact_bottom_number, const double &contact_top_number, const std::vector<ContactPtr> &contacts, ContactOptimizationSolver &solver)
  {
    auto eq_constraint = std::make_shared<ConstraintEquality>();
    // eq_constraint.
    Eigen::MatrixXd A;
    Eigen::VectorXd b;

    // TODO: gravity

    A.setZero(6, contact_number * 6);
    b.setZero(6);
    // b.head<3>() = model_->getTransform().linear() *
    //               Eigen::Vector3d(0, 0, 9.8) * model_->getMass(); // TODO: Check this
    b.head<3>() =  Eigen::Vector3d(0, 0, 9.8) * model_->getMass(); // TODO: Check this
    // TODO: Momentum + b(3~5)
    
    for (int i = 0; i < contact_number; i++)
    {
      A.block<3, 3>(0, i * 6).setIdentity();
      A.block<3, 3>(3, i * 6) = cross_skew(object_init_ori_*object_rotation_*contacts[i]->getContactTransform().translation());
      A.block<3, 3>(3, i * 6 + 3).setIdentity();
    }

    eq_constraint->setA(A);
    eq_constraint->setEqualityCondition(b);

    solver.addConstraint(eq_constraint);

    // every contact
    auto ineq_constraint = std::make_shared<ConstraintInequality>();

    Eigen::MatrixXd C_all;
    Eigen::VectorXd d_all;
    d_all.resize(12*(contact_number));

    C_all.setZero(12 * (contact_number), 6 * (contact_number));

    Eigen::MatrixXd C_max[6];
    Eigen::VectorXd d_max[6];
    const auto &f_limit = robot_->getForceLimitTop();

    for (int i = 0; i < 6; i++)
    {
      C_max[i].setZero(2, 6);
      C_max[i](0, i) = 1;
      C_max[i](1, i) = -1;

      d_max[i].resize(2);
      d_max[i] = f_limit.col(i);
    }

    Eigen::MatrixXd C_i(12, 6), d_i(12, 1);

    for (int i = 0; i < 6; i++)
    {
      C_i.block<2, 6>(i * 2, 0) = C_max[i];
      d_i.block<2, 1>(i * 2, 0) = d_max[i];
    }

    for (int i =0; i < contact_number; i++)
    {
      Eigen::Matrix<double, 6, 6> R_hat;
      Eigen::Matrix<double, 3, 3> R;
      R = contacts[i]->getContactTransform().linear().transpose()*(object_init_ori_*object_rotation_).transpose();
      R_hat.setZero();
      R_hat.block<3, 3>(0, 0) = R;
      R_hat.block<3, 3>(3, 3) = R;

      C_all.block<12, 6>(i * 12, i * 6) = C_i * R_hat;
      d_all.segment<12>(i * 12) = d_i;
    }

    ineq_constraint->setA(C_all);
    ineq_constraint->setOnlyLowerBound(d_all);
    // ineq_constraint->printCondition();

    solver.addConstraint(ineq_constraint);
    solver.setContactNumber(contact_number);
  }

  bool ContactOptimization::solve()
  {
    ContactOptimizationSolver solver_bottom;
    ContactOptimizationSolver solver_top;

    const int contact_number = model_->getContactNumber();
    
    const int contact_bottom_number = getBottomNumber();
    const int contact_top_number = getTopNumber();
    std::vector<ContactPtr> contacts;
    contacts = model_->getContactRobot();

    if (contact_number == 0)
      return false;

    setTop(contact_number, contact_bottom_number, contact_top_number, contacts, solver_top);
    
    Eigen::VectorXd result_bottom;
    Eigen::VectorXd result_top;
    if (solver_top.solve(result_top))
    {
      int j = 0;
      for (int i=0; i < contact_number; i++)
      {
        if (top_bottom_order_.at(i) == 0)
        {
          contacts[i]->setContactForceTorque(result_top.segment<6>((j) * 6));
          j++;
        }
      }
      Eigen::Vector3d normal_force;
      normal_force = result_top.segment<3>(contact_top_number*6);

      setBottom(contact_number, contact_bottom_number, contacts, normal_force, solver_bottom);

      if(solver_bottom.solve(result_bottom))
      {
        int j = 0;
        for (int i = 0; i < contact_number; i++)
        {
          if (top_bottom_order_.at(i) == 1)
          {
            contacts[i]->setContactForceTorque(result_bottom.segment<6>(j * 6));
            j++;
          }
        }

        // for (int i = contact_bottom_number; i < contacts.size(); i++)
        // {
        //   contacts[i]->setContactForceTorque(result_top.segment<6>(i * 6));
        // }

        // Eigen::MatrixXd d_test;
        // d_test.setZero(6,1);
        // contacts[contact_bottom_number]->setContactForceTorque(d_test);
        return true;
      }
      return false;
    }
    
  }

  bool ContactOptimization::solve_one_body()
  {
    ContactOptimizationSolver solver;

    const int contact_number = model_->getContactNumber();
    
    const int contact_bottom_number = getBottomNumber();
    const int contact_top_number = getTopNumber();

    std::vector<ContactPtr> contacts;
    contacts = model_->getContactRobot();

    if (contact_number == 0)
      return false;

    setConstraints(contact_number, contact_bottom_number, contact_top_number, contacts, solver);

    Eigen::VectorXd result;

    if (solver.solve(result))
    {

      for (int i= 0; i < contact_number; i++ )
      {
        contacts[i]->setContactForceTorque(result.segment<6>((i) * 6));
      }
      return true;
    }
    return false;
  }
} // namespace contact_planner
