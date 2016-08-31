/**
 *    @file VehicleMonitorLibrary/VelocityConstraintChecker.hpp
 */

#ifndef VML__VELOCITY_CONSTRAINT_CHECKER_H_
#define VML__VELOCITY_CONSTRAINT_CHECKER_H_


#include "BaseConstraintChecker.hpp"

namespace VehicleMonitorLibrary {


class VelocityConstraintChecker : public BaseConstraintChecker {
 public:
  VelocityConstraintChecker(double maximum_speed);

  virtual ~VelocityConstraintChecker();

 protected:
  virtual void doCheckConstraint(
      const MotionCaptureSystemFrame& motion_capture_system_frame,
      bool emergency_button_pressed, std::map<std::string, bool>& check_result);

 private:


  double maximum_speed_;

};
}
#endif /* VML__VELOCITY_CONSTRAINT_CHECKER_H_ */
