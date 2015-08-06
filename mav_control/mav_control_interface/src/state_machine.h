/*
 * Copyright (c) 2015, Markus Achtelik, ASL, ETH Zurich, Switzerland
 * You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef STATE_MACHINE_H_
#define STATE_MACHINE_H_

#include <boost/msm/back/state_machine.hpp>
#include <boost/msm/back/mpl_graph_fsm_check.hpp>
#include <boost/msm/front/state_machine_def.hpp>
#include <boost/msm/front/functor_row.hpp>
#include <boost/msm/front/euml/common.hpp>
#include <boost/msm/front/euml/operator.hpp>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>

#include <mav_control_interface/position_controller_interface.h>
#include <mav_control_interface/rc_interface.h>
#include "parameters.h"

namespace mav_control_interface {

namespace state_machine {

namespace msm_front = boost::msm::front;
namespace mpl = boost::mpl;
namespace euml = msm_front::euml;

// Events, best outside class declaration.
struct RcUpdate
{
  RcUpdate(const RcData& _rc_data, bool _is_active, bool _is_on)
      : rc_data(_rc_data),
        is_active(_is_active),
        is_on(_is_on)
  {
  }
  RcData rc_data;
  bool is_active;
  bool is_on;
};

struct ReferenceUpdate
{
  ReferenceUpdate(const mav_msgs::EigenTrajectoryPointDeque& _references)
      : references(_references)
  {
  }

  mav_msgs::EigenTrajectoryPointDeque references;
};

struct OdometryUpdate
{
  OdometryUpdate(const mav_msgs::EigenOdometry& _odometry)
      : odometry(_odometry)
  {
  }

  mav_msgs::EigenOdometry odometry;
};

struct Watchdog {};

class StateMachineDefinition;
//typedef boost::msm::back::state_machine<StateMachineDefinition> StateMachine;
// Comment this in, instead of the above, for compile-time state machine analysis.
// See http://www.boost.org/doc/libs/1_58_0/libs/msm/doc/HTML/ch03s05.html#d0e2624
typedef boost::msm::back::state_machine<StateMachineDefinition, boost::msm::back::mpl_graph_fsm_check> StateMachine;

class StateMachineDefinition : public msm_front::state_machine_def<StateMachineDefinition>
{
 private:
  // States, more convenient to have in state machine.
  struct Inactive;
  struct RemoteControl;
  struct RemoteControlReadyForOdometry;
  struct HaveOdometry;
  struct PositionHold;
  struct RcTeleOp;
  struct ExternalReferenceMode;

  // Actions
  struct SetReferenceAttitude;
  struct SetReferencePosition;
  struct SetReferenceToCurrentPosition;
  struct SetOdometry;
  struct ComputeCommand;
  struct SetReferenceFromRc;

  // Guards
  struct RcModeManual;
  struct RcModePosition;
  struct RcActive;
  struct RcOn;

 public:
  // Define initial state. Boost looks for "initial_state".
  typedef Inactive initial_state;
  typedef int no_exception_thrown;

  // Some convenience typedefs to make the table below more readable:
  typedef msm_front::ActionSequence_<mpl::vector<SetOdometry, ComputeCommand> > SetOdometryAndCompute;
  typedef euml::And_<RcActive, RcModePosition> RcActivePosition;
  typedef euml::Not_<RcModeManual> RcModeNotManual;
  typedef msm_front::none InternalTransition;
  typedef msm_front::none NoAction;
  typedef msm_front::none NoGuard;

  // Now define transition table:
  struct transition_table : boost::mpl::vector<
      //    Start     Event         Next      Action                     Guard
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<Inactive, RcUpdate, RemoteControl, NoAction, euml::And_<RcModeManual, RcOn> >,
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<RemoteControl, RcUpdate, InternalTransition, SetReferenceAttitude, RcModeNotManual >,
      msm_front::Row<RemoteControl, RcUpdate, RemoteControlReadyForOdometry, SetReferenceAttitude, RcModeManual >,
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<RemoteControlReadyForOdometry, RcUpdate, RemoteControl, SetReferenceAttitude, RcModeNotManual >,
      msm_front::Row<RemoteControlReadyForOdometry, RcUpdate, InternalTransition, SetReferenceAttitude, RcModeManual >,
      msm_front::Row<RemoteControlReadyForOdometry, OdometryUpdate, HaveOdometry, SetOdometry, NoGuard >,
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<HaveOdometry, RcUpdate, InternalTransition, SetReferenceAttitude, RcModeManual >,
      msm_front::Row<HaveOdometry, OdometryUpdate, InternalTransition, SetOdometry, NoGuard >,
      msm_front::Row<HaveOdometry, Watchdog, RemoteControl, NoAction, NoGuard >,
      msm_front::Row<HaveOdometry, RcUpdate, PositionHold, SetReferenceToCurrentPosition, RcModePosition >,
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<PositionHold, RcUpdate, RemoteControl, NoAction, RcModeManual>,
      msm_front::Row<PositionHold, RcUpdate, RcTeleOp, NoAction, RcActivePosition >,
      msm_front::Row<PositionHold, OdometryUpdate, InternalTransition, SetOdometryAndCompute, NoGuard>,
      msm_front::Row<PositionHold, ReferenceUpdate, ExternalReferenceMode, SetReferencePosition, NoGuard >,
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<RcTeleOp, RcUpdate, PositionHold, SetReferenceToCurrentPosition, euml::And_<euml::Not_<RcActive>, RcModePosition> >,
      msm_front::Row<RcTeleOp, RcUpdate, RemoteControl, NoAction, RcModeManual>,
      msm_front::Row<RcTeleOp, RcUpdate, InternalTransition, SetReferenceFromRc, RcActivePosition >,
      msm_front::Row<RcTeleOp, OdometryUpdate, InternalTransition, SetOdometryAndCompute, NoGuard>,
      //  +---------+-------------+---------+---------------------------+----------------------+
      msm_front::Row<ExternalReferenceMode, RcUpdate, RemoteControl, NoAction, RcModeManual>,
      msm_front::Row<ExternalReferenceMode, RcUpdate, RcTeleOp, NoAction, RcActivePosition >,
      msm_front::Row<ExternalReferenceMode, ReferenceUpdate, InternalTransition, SetReferencePosition, NoGuard>,
      msm_front::Row<ExternalReferenceMode, OdometryUpdate, InternalTransition, SetOdometryAndCompute, NoGuard>
  //  +---------+-------------+---------+---------------------------+----------------------+
      >
  {
  };

 public:
  StateMachineDefinition(std::shared_ptr<PositionControllerInterface> controller,
                         ros::Publisher& command_publisher);

  template<class Event, class FSM>
  void on_entry(Event const&, FSM&)
  {
    std::cout << "entering StateMachine" << std::endl;
  }

  template<class Event, class FSM>
  void on_exit(Event const&, FSM&)
  {
    std::cout << "leaving StateMachine" << std::endl;
  }

  bool GetVerbose() const;
  void SetVerbose(bool verbose);
  void SetParameters(const Parameters& parameters);

private:
  bool verbose_;
  std::shared_ptr<PositionControllerInterface> controller_;
  ros::Publisher command_publisher_;
  Parameters parameters_;
  mav_msgs::EigenOdometry current_state_;
  mav_msgs::EigenTrajectoryPointDeque current_reference_queue_;

  void PublishAttitudeCommand(const mav_msgs::EigenRollPitchYawrateThrust& command) const;

  // Implementation of state machine:

  // States
  struct Inactive : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.GetVerbose())
        std::cout << "[State]: Inactive" << std::endl;
    }
  };

  struct RemoteControl : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.GetVerbose())
        std::cout << "[State]: RemoteControl" << std::endl;
    }
  };

  struct RemoteControlReadyForOdometry : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.GetVerbose())
        std::cout << "[State]: RemoteControlReadyForOdometry" << std::endl;
    }
  };

  struct HaveOdometry : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.GetVerbose())
        std::cout << "[State]: HaveOdometry" << std::endl;
    }
  };
  struct PositionHold : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.GetVerbose())
        std::cout << "[State]: PositionHold" << std::endl;
    }
  };

  struct RcTeleOp : public msm_front::state<>
  {
    template<class FSM>
    void on_entry(const RcUpdate& evt, FSM& fsm)
    {
      last_iteration_time_ = evt.rc_data.timestamp;
      if (fsm.GetVerbose())
        std::cout << "[State]: RcTeleOp" << std::endl;
    }
    ros::Time last_iteration_time_;
  };

  struct ExternalReferenceMode : public msm_front::state<>
  {
    template<class Event, class FSM>
    void on_entry(Event const& evt, FSM& fsm)
    {
      if (fsm.GetVerbose())
        std::cout << "[State]: ExternalReferenceMode" << std::endl;
    }
  };

  // Actions
  struct SetReferenceAttitude
  {
    template<class FSM, class SourceState, class TargetState>
    void operator()(const RcUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      mav_msgs::EigenRollPitchYawrateThrust command;
      command.pitch = evt.rc_data.right_up_down * fsm.parameters_.rc_max_roll_pitch_command_;
      command.roll = evt.rc_data.right_side * fsm.parameters_.rc_max_roll_pitch_command_;
      command.yaw_rate = -evt.rc_data.left_side * fsm.parameters_.rc_max_yaw_rate_command_;
      constexpr double thrust_below_hovering_factor = 0.8;
      command.thrust.z() = (evt.rc_data.left_up_down + 1.0) * fsm.controller_->getMass() * 9.81 * thrust_below_hovering_factor;
      fsm.PublishAttitudeCommand(command);
    }
  };

  struct SetReferencePosition
  {
    template<class FSM, class SourceState, class TargetState>
    void operator()(const ReferenceUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      fsm.controller_->setReferenceArray(evt.references);
      fsm.current_reference_queue_ = evt.references;
    }
  };

  struct SetReferenceToCurrentPosition
  {
    template<class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& evt, FSM& fsm, SourceState&, TargetState&)
    {
      mav_msgs::EigenTrajectoryPoint reference;
      reference.position_W = fsm.current_state_.position_W;
      reference.setFromYaw(mav_msgs::yawFromQuaternion(fsm.current_state_.orientation_W_B));

      fsm.controller_->setReference(reference);
      fsm.current_reference_queue_.clear();
      fsm.current_reference_queue_.push_back(reference);
    }
  };

  struct SetOdometry
  {
    template<class FSM, class SourceState, class TargetState>
    void operator()(const OdometryUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      fsm.current_state_ = evt.odometry;
      fsm.controller_->setOdometry(evt.odometry);
    }
  };

  struct ComputeCommand
  {
    template<class EVT, class FSM, class SourceState, class TargetState>
    void operator()(EVT const& evt, FSM& fsm, SourceState&, TargetState&)
    {
      mav_msgs::EigenRollPitchYawrateThrust command;
      fsm.controller_->calculateRollPitchYawrateThrustCommand(&command);
      fsm.PublishAttitudeCommand(command);
    }
  };

  struct SetReferenceFromRc
  {
    template<class FSM>
    void operator()(const RcUpdate& evt, FSM& fsm, RcTeleOp& src_state, RcTeleOp&)
    {
      const double dt = (evt.rc_data.timestamp - src_state.last_iteration_time_).toSec();

      const Parameters& p = fsm.parameters_;
      const RcData& rc_data = evt.rc_data;

      mav_msgs::EigenTrajectoryPoint new_reference;
      mav_msgs::EigenTrajectoryPoint current_reference = fsm.current_reference_queue_.front();

      Eigen::Vector3d stick_position;
      stick_position.x() = p.stick_deadzone_(rc_data.right_up_down);
      stick_position.y() = p.stick_deadzone_(-rc_data.right_side);
      stick_position.z() = p.stick_deadzone_(rc_data.left_up_down);

      const double yaw = mav_msgs::yawFromQuaternion(fsm.current_state_.orientation_W_B);
      Eigen::Vector3d desired_velocities = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * stick_position;
      desired_velocities *= p.rc_teleop_max_velocity_;
      const double absolute_velocity = desired_velocities.norm();
      if (absolute_velocity > p.rc_teleop_max_velocity_) {
        desired_velocities = desired_velocities / absolute_velocity * p.rc_teleop_max_velocity_;
      }

      Eigen::Vector3d candidate_reference_position = current_reference.position_W + desired_velocities * dt;
      const double distance_to_current_position = (candidate_reference_position
          - fsm.current_state_.position_W).norm();
      if (distance_to_current_position > p.rc_teleop_max_carrot_distance_) {
        new_reference.position_W = current_reference.position_W;
        ROS_WARN_THROTTLE(1, "New position reference is to far (%f m) away from the current position.",
                          distance_to_current_position);
      }
      else {
        new_reference.position_W = candidate_reference_position;
      }

      double new_yaw = current_reference.getYaw()
          + p.stick_deadzone_(-rc_data.left_side) * p.rc_max_yaw_rate_command_ * dt;
      if (new_yaw > M_PI) {
        new_yaw -= 2.0 * M_PI;
      }
      else if (new_yaw < -M_PI) {
        new_yaw += 2.0 * M_PI;
      }

      new_reference.setFromYaw(new_yaw);

      fsm.current_reference_queue_.clear();
      fsm.current_reference_queue_.push_back(new_reference);
      fsm.controller_->setReference(new_reference);

      src_state.last_iteration_time_ = evt.rc_data.timestamp;
    }
  };

  // Guards
  struct RcModeManual
  {
    template<class FSM, class SourceState, class TargetState>
    bool operator()(const RcUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      const bool rc_mode_manual = evt.rc_data.control_mode == RcData::ControlMode::MANUAL;
      return rc_mode_manual;
    }
  };

  struct RcModePosition
  {
    template<class FSM, class SourceState, class TargetState>
    bool operator()(const RcUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      const bool rc_mode_position = evt.rc_data.control_mode == RcData::ControlMode::POSITION_CONTROL;
      return rc_mode_position;
    }
  };

  struct RcActive
  {
    template<class FSM, class SourceState, class TargetState>
    bool operator()(const RcUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      return evt.is_active;
    }
  };

  struct RcOn
  {
    template<class FSM, class SourceState, class TargetState>
    bool operator()(const RcUpdate& evt, FSM& fsm, SourceState&, TargetState&)
    {
      return evt.is_on;
    }
  };

}; // end class StateMachine_

} // end namespace state_machine

} // namespace mav_control_interface

#endif /* STATE_MACHINE_H_ */