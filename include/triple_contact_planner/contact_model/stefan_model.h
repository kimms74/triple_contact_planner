#ifndef STEFAN_MODEL_H
#define STEFAN_MODEL_H

#include <memory>
#include <vector>
#include <Eigen/Dense>

#include "triple_contact_planner/contact/contact.h"
#include "triple_contact_planner/contact_model/contact_model.h"

namespace contact_planner
{

/**
 * @brief The ContactModel class
 */
class StefanModel : public ContactModel 
{
public:
  StefanModel() { setMass(3.75); }
  StefanModel(std::string name) : ContactModel(name) { setMass(3.75); }

  void contactWrenchOptimize();

  /**
   * @brief createContactSamples
   */
  virtual void createContactSamples(std::vector<ContactPtr> &contact_samples){}

  /**
   * @brief operation
   * @param dir Direction we want to move
   * @param delta
   * @return Whether it is possible operation.
   */
  virtual bool operate(OperationDirection dir, double delta_x, double delta_orientation){}

  /**
   * @brief isSamePose
   * @param model An model to be compared
   * @param threshold_x Linear threshold
   * @param threshold_orientation Orientation threshold
   * @return
   */
  bool isSamePose(const ContactModel& model, double threshold_x, double threshold_orientation){}

  virtual ContactPtr getBottomContact(){};

  virtual double getBottomPositionZ(){};

protected:
};

// typedef std::shared_ptr<ContactModel> ContactModelPtr;

}

#endif
