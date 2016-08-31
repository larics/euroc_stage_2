/**
 *    @file VelocityConstraintChecker.cpp
 */

#include "vehicle_monitor_library/VelocityConstraintChecker.hpp"
#include "vehicle_monitor_library/Vehicle.hpp"

using namespace std;

namespace VehicleMonitorLibrary {

VelocityConstraintChecker::VelocityConstraintChecker(double maximum_speed)
: BaseConstraintChecker("VELOCITY_CONSTRAINT_CHECKER"),
  maximum_speed_(maximum_speed) {}

VelocityConstraintChecker::~VelocityConstraintChecker() {}

void VelocityConstraintChecker::doCheckConstraint(
    const MotionCaptureSystemFrame& motion_capture_system_frame,
    bool emergency_button_pressed, std::map<std::string, bool>& check_result) {

  bool constraint_ok;

  for (const std::pair<std::string, Vehicle::Ptr>& vehicle_map_element :
      *vehicles_map_) {
    const std::string& vehicle_id = vehicle_map_element.first;
    Vehicle::Ptr vehicle = vehicle_map_element.second;
    if (vehicle->getHasNewState() == false) {
      continue;
    }

    VehicleState estimated_state;

    if (vehicle->getState(&estimated_state) == false) {
      continue;
    }

    constraint_ok = true;

    if (estimated_state.velocity.norm() > maximum_speed_) {
      constraint_ok = false;
    }

    check_result.insert(make_pair(vehicle_id, constraint_ok));
  }
}

}
