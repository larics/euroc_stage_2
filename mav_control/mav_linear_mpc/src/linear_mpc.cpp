/*
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 *
 * This code can be used only for academic, non-commercial use.
 * This code cannot be redistributed under any license, open source or otherwise.
 *
 * CVXGEN license: http://cvxgen.com/docs/license.html
 * FORCES license: http://forces.ethz.ch
 *
 */

#include <mav_linear_mpc/linear_mpc.h>

namespace mav_control {

LinearModelPredictiveController::LinearModelPredictiveController(ros::NodeHandle& nh,
                                                                 ros::NodeHandle& private_nh)
    : nh_(nh),
      private_nh_(private_nh),
      controller_nh_(private_nh, "controller"),
      gravity_(9.81),
      sampling_time_(0.01),
      height_error_integrator_(0.0),
      initialized_params_(false),
      command_roll_pitch_yaw_thrust_(0,0,0,0),
      linearized_command_roll_pitch_thrust_(0,0,0),
      SolveTime_avg(0)
{
  this->InitializeParams();
}

LinearModelPredictiveController::~LinearModelPredictiveController()
{
}

void LinearModelPredictiveController::InitializeParams()
{
  state_observer_.reset(new KFDisturbanceObserver(nh_, private_nh_));
  steady_state_calculation_.reset(new SteadyStateCalculation(nh_, private_nh_));

  //Get parameters from RosParam server
  std::vector<double> temporary_A, temporary_B, temporary_Bd, temporary_umin, temporary_umax;

  controller_nh_.param<bool>("verbose", verbose_, false);

  if (!controller_nh_.getParam("mass", mass_))
    ROS_ERROR("mass in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("n_state", state_size_))
    ROS_ERROR("state_size_ in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("n_input", input_size_))
    ROS_ERROR("input_size_ in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("n_disturbance", disturbance_size_))
    ROS_ERROR("disturbance_size_ in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("n_measurement", measurement_size_))
    ROS_ERROR("measurement_size_ in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("height_error_integration_limit", height_error_integrator_limit_))
    ROS_ERROR("height_error_integration_limit in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("use_height_error_integrator", use_height_error_integrator_))
      ROS_ERROR("use_height_error_integrator in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("use_KF_estimated_state", use_KF_estimated_state_))
      ROS_ERROR("use_KF_estimated_state in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("use_xy_offset_free", use_xy_offset_free_))
        ROS_ERROR("use_xy_offset_free in MPC is not loaded from ros parameter server");


  if (!controller_nh_.getParam("A", temporary_A))
    ROS_ERROR("A in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("B", temporary_B))
    ROS_ERROR("B in MPC is not loaded from ros parameter server");

  if (!controller_nh_.getParam("Bd", temporary_Bd))
    ROS_ERROR("Bd in MPC is not loaded from ros parameter server");

  Eigen::Map<Eigen::MatrixXd> A_map(temporary_A.data(), state_size_, state_size_);
  Eigen::Map<Eigen::MatrixXd> B_map(temporary_B.data(), state_size_, input_size_);
  Eigen::Map<Eigen::MatrixXd> Bd_map(temporary_Bd.data(), state_size_, disturbance_size_);

  model_A_ = A_map;
  model_B_ = B_map;
  model_Bd_ = Bd_map;

#ifdef UseCVXGENSolver
  //CVXGEN initialization
  set_defaults();
  setup_indexing();

  //CVXGEN settings
  settings.verbose = 0;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.A), state_size_, state_size_) = model_A_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.B), state_size_, input_size_) = model_B_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Bd), state_size_, disturbance_size_) = model_Bd_;

#endif

  Eigen::Vector3d initial_rpy;
  quat2rpy(odometry_.orientation_W_B, &initial_rpy);

  state_observer_->Reset(odometry_.position_W, odometry_.getVelocityWorld(), initial_rpy,
                         odometry_.angular_velocity_B, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  initialized_params_ = true;

  ROS_INFO("MPC initialized correctly");
}

void LinearModelPredictiveController::ApplyParameters()
{

  //Compute terminal cost
  //Q_final(k+1) = A'*Q_final(k)*A - (A'*Q_final(k)*B)*inv(B'*Q_final(k)*B+R)*(B'*Q_final(k)*A)+ Q;
  Q_final_ = Q_;
  for (int i = 0; i < 1000; i++) {
    Eigen::MatrixXd temp = (model_B_.transpose() * Q_final_ * model_B_ + R_);
    Q_final_ = model_A_.transpose() * Q_final_ * model_A_
        - (model_A_.transpose() * Q_final_ * model_B_) * temp.inverse()
            * (model_B_.transpose() * Q_final_ * model_A_) + Q_;
  }

  Eigen::MatrixXd temporary_matrix = model_B_.transpose() * Q_final_ * model_B_ + R_;
  LQR_K_ = temporary_matrix.inverse() * (model_B_.transpose() * Q_final_ * model_A_);

#ifdef UseForcesSolver
  cost_Hessian_ = Eigen::MatrixXd::Zero(state_size_ + disturbance_size_ + 2 * input_size_,
                                        state_size_ + disturbance_size_ + 2 * input_size_);
  cost_Hessian_.block(0, 0, state_size_, state_size_) = 2.0 * Q_;
  cost_Hessian_.block(state_size_ + disturbance_size_ + input_size_,
                      state_size_ + disturbance_size_ + input_size_, input_size_, input_size_) = 2.0
      * (R_ + R_delta_);
  cost_Hessian_.block(state_size_ + disturbance_size_, state_size_ + disturbance_size_, input_size_,
                      input_size_) = 2.0 * R_delta_;
  cost_Hessian_.block(state_size_ + disturbance_size_ + input_size_,
                      state_size_ + disturbance_size_, input_size_, input_size_) = -2.0 * R_delta_;
  cost_Hessian_.block(state_size_ + disturbance_size_,
                      state_size_ + disturbance_size_ + input_size_, input_size_, input_size_) =
      -2.0 * R_delta_;

  ROS_INFO_STREAM("H = " << std::endl << cost_Hessian_);
  cost_Hessian_final_ = Eigen::MatrixXd::Zero(state_size_ + disturbance_size_ + input_size_,
                                              state_size_ + disturbance_size_ + input_size_);
  cost_Hessian_final_.block(0, 0, state_size_, state_size_) = 2.0 * Q_final_;

  ROS_INFO_STREAM("H_N = " << std::endl << cost_Hessian_final_);
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(FORCES_params_.H_N),
                              state_size_ + disturbance_size_ + input_size_,
                              state_size_ + disturbance_size_ + input_size_) = cost_Hessian_final_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(FORCES_params_.H),
                              state_size_ + disturbance_size_ + 2 * input_size_,
                              state_size_ + disturbance_size_ + 2 * input_size_) = cost_Hessian_;

  Eigen::VectorXd Forces_umin(2 * input_size_);
  Eigen::VectorXd Forces_umax(2 * input_size_);
  Forces_umin << u_min_, u_min_;
  Forces_umax << u_max_, u_max_;

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(FORCES_params_.umin), 2 * input_size_, 1) =
      Forces_umin;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(FORCES_params_.umax), 2 * input_size_, 1) =
      Forces_umax;
#endif

#ifdef UseCVXGENSolver

  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q), state_size_, state_size_) = Q_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.Q_final), state_size_, state_size_) = Q_final_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R), input_size_, input_size_) = R_;
  Eigen::Map<Eigen::MatrixXd>(const_cast<double*>(params.R_omega), input_size_, input_size_) = R_delta_;

  for (int i = 0; i < input_size_; i++) {
    params.u_max[i] = u_max_(i);
    params.u_min[i] = u_min_(i);
  }

#endif

  ROS_INFO("Tuning parameters updated...");

  ROS_INFO_STREAM("diag(Q) = \n" << Q_.diagonal());
  ROS_INFO_STREAM("diag(R) = \n" << R_.diagonal());
  ROS_INFO_STREAM("diag(R_delta) = \n " << R_delta_.diagonal());
  ROS_INFO_STREAM("Q_final = \n" << Q_final_);
}

// TODO(fmina/acmarkus): move this to state machine
void LinearModelPredictiveController::SetOdometry(const mav_msgs::EigenOdometry& odometry)
{
  static mav_msgs::EigenOdometry previous_odometry = odometry;

  if (odometry.position_W.allFinite() == false) {
    odometry_.position_W = previous_odometry.position_W;
    ROS_WARN("Odometry.position has a non finite element");
  } else {
    odometry_.position_W = odometry.position_W;
    previous_odometry.position_W = odometry.position_W;
  }

  if (odometry.velocity_B.allFinite() == false) {
    odometry_.velocity_B = previous_odometry.velocity_B;
    ROS_WARN("Odometry.velocity has a non finite element");
  } else {
    odometry_.velocity_B = odometry.velocity_B;
    previous_odometry.velocity_B = odometry.velocity_B;
  }

  if (odometry.angular_velocity_B.allFinite() == false) {
    odometry_.angular_velocity_B = previous_odometry.angular_velocity_B;
    ROS_WARN("Odometry.angular_velocity has a non finite element");
  } else {
    odometry_.angular_velocity_B = odometry.angular_velocity_B;
    previous_odometry.angular_velocity_B = odometry.angular_velocity_B;
  }

  odometry_.orientation_W_B = odometry.orientation_W_B;
  previous_odometry.orientation_W_B = odometry.orientation_W_B;
}

void LinearModelPredictiveController::SetCommandTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory)
{
  //clear queue
  position_command_queue_.clear();
  velocity_command_queue_.clear();
  yaw_command_queue_.clear();

  command_trajectory_ = command_trajectory;

  for (int i = 0; i < PREDICTION_HORIZON; i++) {
    position_command_queue_.push_back(command_trajectory.position_W);
    velocity_command_queue_.push_back(command_trajectory.velocity_W);
    yaw_command_queue_.push_back(command_trajectory.getYaw());
  }
}

void LinearModelPredictiveController::SetCommandTrajectory(
    const mav_msgs::EigenTrajectoryPointDeque& command_trajectory_array)
{
  int array_size = command_trajectory_array.size();
  if (array_size < 1) {
    return;
  }

  command_trajectory_ = command_trajectory_array.front();

  position_command_queue_.clear();
  velocity_command_queue_.clear();
  yaw_command_queue_.clear();

  for (int i = 0; i < array_size; i++) {
    position_command_queue_.push_back(command_trajectory_array.at(i).position_W);
    velocity_command_queue_.push_back(command_trajectory_array.at(i).velocity_W);
    yaw_command_queue_.push_back(command_trajectory_array.at(i).getYaw());
  }

  int i = array_size;
  while (i < PREDICTION_HORIZON) {
    position_command_queue_.push_back(position_command_queue_.back());
    velocity_command_queue_.push_back(velocity_command_queue_.back());
    yaw_command_queue_.push_back(yaw_command_queue_.back());
    i++;
  }
}

void LinearModelPredictiveController::UpdateQueue(Eigen::VectorXd estimated_disturbances)
{
  if (position_command_queue_.empty()) {
    ROS_WARN("Update queue empty");
    return;
  }

  const Eigen::Vector3d last_position_in_queue = position_command_queue_.back();
  const Eigen::Vector3d last_velocity_in_queue = velocity_command_queue_.back();
  const double last_yaw_in_queue = yaw_command_queue_.back();
  while (position_command_queue_.size() < PREDICTION_HORIZON) {
    position_command_queue_.push_back(last_position_in_queue);
    velocity_command_queue_.push_back(last_velocity_in_queue);
    yaw_command_queue_.push_back(last_yaw_in_queue);
  }

  Eigen::VectorXd target_state(state_size_);
  Eigen::VectorXd target_input(input_size_);
  Eigen::VectorXd reference(6);
  Eigen::Vector3d position = position_command_queue_.front();
  double yaw = yaw_command_queue_.front();

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  transform.setRotation(q);
  br.sendTransform(
      tf::StampedTransform(transform, ros::Time::now(), "world", "controller_reference"));

#ifdef UseForcesSolver
  Eigen::VectorXd z_ss(state_size_ + disturbance_size_ + 2 * input_size_);
  Eigen::VectorXd z_ss_final(state_size_ + disturbance_size_ + input_size_);
  Eigen::VectorXd f_final(state_size_ + disturbance_size_ + input_size_);

  if (FORCES_queue_.empty()) {
    for (int i = 0; i < PREDICTION_HORIZON - 1; i++) {
      reference << position_command_queue_.front(), velocity_command_queue_.front();
      position_command_queue_.pop_front();
      velocity_command_queue_.pop_front();
      steady_state_calculation_->ComputeSteadyState(estimated_disturbances, reference,
                                                    &target_state, &target_input);

      z_ss << target_state, Eigen::VectorXd::Zero(disturbance_size_ + input_size_), target_input;
      Eigen::VectorXd tmp_vector(state_size_ + disturbance_size_ + 2 * input_size_);
      tmp_vector = -cost_Hessian_.transpose() * z_ss;
      FORCES_queue_.push_back(tmp_vector);
    }
    reference << position_command_queue_.front(), velocity_command_queue_.front();

    steady_state_calculation_->ComputeSteadyState(estimated_disturbances, reference, &target_state,
                                                  &target_input);
    z_ss_final << target_state, Eigen::VectorXd::Zero(disturbance_size_ + input_size_);
    //  f_final.resize(state_size_ + disturbance_size_ + input_size_);
    f_final = -cost_Hessian_final_.transpose() * z_ss_final;
  } else {

    reference << position_command_queue_.front(), velocity_command_queue_.front();
    position_command_queue_.pop_front();
    velocity_command_queue_.pop_front();

    steady_state_calculation_->ComputeSteadyState(estimated_disturbances, reference, &target_state,
                                                  &target_input);

    z_ss << target_state, Eigen::VectorXd::Zero(disturbance_size_ + input_size_), target_input;
    z_ss_final << target_state, Eigen::VectorXd::Zero(disturbance_size_ + input_size_);
    f_final = -cost_Hessian_final_.transpose() * z_ss_final;

//    if (position_command_queue_.size() < 2) {
//      FORCES_queue_.clear();
//      for (int i = 0; i < PREDICTION_HORIZON - 1; i++) {
//        Eigen::VectorXd tmp_vector(state_size_ + disturbance_size_ + 2 * input_size_);
//        tmp_vector = -cost_Hessian_.transpose() * z_ss;
//        FORCES_queue_.push_back(tmp_vector);
//      }
//
//    }
//    else {

      Eigen::VectorXd tmp_vector(state_size_ + disturbance_size_ + 2 * input_size_);
      FORCES_queue_.push_back(tmp_vector);
      FORCES_queue_.back() = -cost_Hessian_.transpose() * z_ss;
      FORCES_queue_.pop_front();
//    }
  }
  //For different prediction horizon, change this accordingly!
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_1),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[0];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_2),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[1];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_3),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[2];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_4),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[3];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_5),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[4];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_6),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[5];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_7),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[6];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_8),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[7];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_9),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[8];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_10),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[9];

  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_11),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[10];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_12),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[11];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_13),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[12];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_14),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[13];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_15),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[14];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_16),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[15];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_17),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[16];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_18),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[17];
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_19),
                              state_size_ + disturbance_size_ + 2 * input_size_, 1) =
      FORCES_queue_[18];

  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.f_N),
                              state_size_ + disturbance_size_ + input_size_, 1) = f_final;

#endif

#ifdef UseCVXGENSolver
  if(CVXGEN_queue_.empty()) {
    for (int i = 0; i < PREDICTION_HORIZON; i++) {
      reference << position_command_queue_.front(), velocity_command_queue_.front();
      position_command_queue_.pop_front();
      velocity_command_queue_.pop_front();

      steady_state_calculation_->ComputeSteadyState(estimated_disturbances, reference,
          &target_state, &target_input);

      CVXGEN_queue_.push_back(target_state);
    }
  } else {

    if(position_command_queue_.size() < 1) {
      return;
    }

    reference << position_command_queue_.front(), velocity_command_queue_.front();
    position_command_queue_.pop_front();
    velocity_command_queue_.pop_front();

    steady_state_calculation_->ComputeSteadyState(estimated_disturbances, reference,
        &target_state, &target_input);

//    if(position_command_queue_.size() < 2) {
//      CVXGEN_queue_.clear();
//      for(int i=0; i<PREDICTION_HORIZON; i++) {
//        CVXGEN_queue_.push_back(target_state);
//      }
//
//    } else {

      CVXGEN_queue_.push_back(target_state);
      CVXGEN_queue_.pop_front();
//    }
  }
  for (int i = 0; i < PREDICTION_HORIZON; i++) {
    Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_ss[i]), state_size_, 1) = CVXGEN_queue_[i];
  }
#endif

  yaw_command_ = yaw_command_queue_.front();
  yaw_command_queue_.pop_front();
}

void LinearModelPredictiveController::CalculateAttitudeThrust(Eigen::Vector4d *ref_attitude_thrust)
{
  assert(ref_attitude_thrust != nullptr);
  assert(initialized_params_ != nullptr);
  ros::WallTime starting_time = ros::WallTime::now();

  //Declare variables


  Eigen::VectorXd reference(6);

  reference << command_trajectory_.position_W, command_trajectory_.velocity_W;

  Eigen::Matrix3d R_rot;
  Eigen::VectorXd KF_estimated_state;
  Eigen::Vector2d roll_pitch_inertial_frame;
  Eigen::VectorXd target_state(state_size_);
  Eigen::VectorXd target_input(input_size_);

  Eigen::VectorXd estimated_disturbances(disturbance_size_);
  Eigen::VectorXd x_0(state_size_);

  double yaw_rate_cmd;
  double roll;
  double pitch;
  double yaw;

#ifdef UseForcesSolver
  Eigen::VectorXd z1(2 * (state_size_ + disturbance_size_ + input_size_));

  FireFlyOffsetFreeMPC_output output;
  FireFlyOffsetFreeMPC_info info;

#endif

  R_rot = odometry_.orientation_W_B.toRotationMatrix();

  Eigen::Vector3d velocity_W = R_rot * odometry_.velocity_B;

  state_observer_->FeedAttitudeCommand(command_roll_pitch_yaw_thrust_);
  state_observer_->FeedPositionMeasurement(odometry_.position_W);
  state_observer_->FeedVelocityMeasurement(velocity_W);
  state_observer_->FeedRotationMatrix(R_rot);

  bool observer_update_successful = state_observer_->UpdateEstimator();
  if(!observer_update_successful) {
    Eigen::Vector3d attitude;
    quat2rpy(odometry_.orientation_W_B, &attitude);
    state_observer_->Reset(odometry_.position_W, velocity_W, attitude, Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  state_observer_->GetEstimatedState(&KF_estimated_state);

  if (use_xy_offset_free_ == true) {
    estimated_disturbances = KF_estimated_state.segment(12, disturbance_size_);
//    estimated_disturbances(2) = 0;
  } else {
    estimated_disturbances.setZero(disturbance_size_);
  }

  if (use_height_error_integrator_) {
    height_error_integrator_ += sampling_time_
        * (command_trajectory_.position_W(2) - odometry_.position_W(2));
    if (height_error_integrator_ > height_error_integrator_limit_) {
      height_error_integrator_ = height_error_integrator_limit_;
    }
    if (height_error_integrator_ < -height_error_integrator_limit_) {
      height_error_integrator_ = -height_error_integrator_limit_;
    }
  } else {
    height_error_integrator_ = 0.0;
  }

  if (use_KF_estimated_state_ == true) {

    roll = KF_estimated_state(6);
    pitch = KF_estimated_state(7);
    yaw = KF_estimated_state(8);

    roll_pitch_inertial_frame << -sin(yaw) * pitch + cos(yaw) * roll, cos(yaw) * pitch + sin(yaw) * roll;
    x_0 << KF_estimated_state.segment(0, 6), roll_pitch_inertial_frame;

  } else {
    Eigen::Vector3d current_roll_pitch_yaw;
    quat2rpy(odometry_.orientation_W_B, &current_roll_pitch_yaw);

    roll = current_roll_pitch_yaw(0);
    pitch = current_roll_pitch_yaw(1);
    yaw = current_roll_pitch_yaw(2);

    roll_pitch_inertial_frame << -sin(yaw) * pitch + cos(yaw) * roll, cos(yaw) * pitch + sin(yaw) * roll;
    x_0 << odometry_.position_W, velocity_W, roll_pitch_inertial_frame;
  }

  steady_state_calculation_->ComputeSteadyState(estimated_disturbances, reference, &target_state,
                                                &target_input);

  UpdateQueue(estimated_disturbances);

#ifdef UseForcesSolver
  //Solve using FORCES

  z1 << x_0, estimated_disturbances, linearized_command_roll_pitch_thrust_, Eigen::VectorXd::Zero(
      state_size_ + disturbance_size_ + input_size_, 1);

  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(FORCES_params_.z1),
                              2 * (state_size_ + disturbance_size_ + input_size_), 1) = z1;

  FireFlyOffsetFreeMPC_solve(const_cast<FireFlyOffsetFreeMPC_params*>(&FORCES_params_), &output,
                             &info);

  linearized_command_roll_pitch_thrust_ << output.output_1[0], output.output_1[1], output.output_1[2];

  SolveTime_avg += info.solvetime;

  int iteration_number = info.it;

#endif

#ifdef UseCVXGENSolver
  //Solve using CVXGEN
  ROS_INFO_ONCE("using CVXGEN");

  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.d), disturbance_size_, 1) = estimated_disturbances;
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.u_prev), input_size_, 1) =
  linearized_command_roll_pitch_thrust_;
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.x_0), state_size_, 1) = x_0;
  Eigen::Map<Eigen::VectorXd>(const_cast<double*>(params.u_ss), input_size_, 1) = target_input;

  tic();
  int iteration_number = solve();
  SolveTime_avg += tocq();

  linearized_command_roll_pitch_thrust_ << vars.u_0[0], vars.u_0[1], vars.u_0[2];

#endif

  if (iteration_number < 0) {  //Other ways to check solver failure??
//   = false;
    linearized_command_roll_pitch_thrust_ = LQR_K_ * (target_state - x_0);
    linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMax(u_min_);
    linearized_command_roll_pitch_thrust_ = linearized_command_roll_pitch_thrust_.cwiseMin(u_max_);
  }

  command_roll_pitch_yaw_thrust_(3) = (linearized_command_roll_pitch_thrust_(2) + gravity_
      + Ki_height_ * height_error_integrator_) / (cos(roll) * cos(pitch));
  double ux = linearized_command_roll_pitch_thrust_(1) * (gravity_ / command_roll_pitch_yaw_thrust_(3));
  double uy = linearized_command_roll_pitch_thrust_(0) * (gravity_ / command_roll_pitch_yaw_thrust_(3));

  command_roll_pitch_yaw_thrust_(0) = ux * sin(yaw) + uy * cos(yaw);
  command_roll_pitch_yaw_thrust_(1) = ux * cos(yaw) - uy * sin(yaw);
  command_roll_pitch_yaw_thrust_(2) = yaw_command_;

  int n_rotations = (int) (yaw / (2.0 * M_PI));
  yaw = yaw - 2.0 * M_PI * n_rotations;

  if (yaw > M_PI)
    yaw = M_PI - yaw;

  double yaw_error = command_roll_pitch_yaw_thrust_(2) - yaw;

  if (std::abs(yaw_error) > M_PI) {
    if (yaw_error > 0.0)
      yaw_error = yaw_error - 2.0 * M_PI;
    else
      yaw_error = yaw_error + 2.0 * M_PI;
  }

  yaw_rate_cmd = K_yaw_ * yaw_error;

  if (yaw_rate_cmd > M_PI_2)
    yaw_rate_cmd = M_PI_2;

  if (yaw_rate_cmd < -M_PI_2)
    yaw_rate_cmd = -M_PI_2;

  *ref_attitude_thrust << command_roll_pitch_yaw_thrust_(0), command_roll_pitch_yaw_thrust_(1), yaw_rate_cmd, command_roll_pitch_yaw_thrust_(
      3) * mass_;  //[N]

  double diff_time = (ros::WallTime::now() - starting_time).toSec();

  if (verbose_) {
    static int counter = 0;
    if (counter > 100) {
      printf("average solve time = %.5f\n", SolveTime_avg / counter);
      SolveTime_avg = 0.0;

      std::cout << "odometry position : \t" << odometry_.position_W(0) << "\t"
          << odometry_.position_W(1) << "\t" << odometry_.position_W(2) << std::endl;

      std::cout << "Controller loop time : " << diff_time << " sec" << std::endl;

      std::cout << "roll ref : \t" << command_roll_pitch_yaw_thrust_(0) << "\t" << "pitch ref : \t"
          << command_roll_pitch_yaw_thrust_(1) << "\t" << "yaw ref : \t"
          << command_roll_pitch_yaw_thrust_(2) << "\t" << "thrust ref : \t"
          << command_roll_pitch_yaw_thrust_(3) << "\t" << "yawrate ref : \t" << yaw_rate_cmd
          << std::endl;
      counter = 0;
    }
    counter++;
  }

}

void LinearModelPredictiveController::SetMaximumCommand(double max_roll, double max_pitch, double max_thrust)
{
  const double kMaxThrust = 0.5 * gravity_;
  if (max_thrust > kMaxThrust){
    max_thrust = kMaxThrust;
    ROS_WARN("Reduced max thrust from %f to %f", max_thrust, kMaxThrust);
  }

  if (max_thrust < 0){
    max_thrust = kMaxThrust;
    ROS_FATAL("max_thrust < 0 -- wtf?!?");
  }

  const double kMaxRollPitch = 60.0 / 180.0 * M_PI;  // 60 deg should be enough by far.
  if (max_roll > kMaxRollPitch) {
    max_roll = kMaxRollPitch;
    ROS_WARN("Reduced max roll from %f to %f. Make sure the angle is given in rad", max_roll, kMaxRollPitch);
  }

  if (max_pitch > kMaxRollPitch) {
    max_pitch = kMaxRollPitch;
    ROS_WARN("Reduced max pitch from %f to %f. Make sure the angle is given in rad", max_pitch, kMaxRollPitch);
  }

  u_max_ << max_roll, max_pitch, max_thrust;
  u_min_ << -max_roll, -max_pitch, -max_thrust;
}

bool LinearModelPredictiveController::GetCurrentReference(mav_msgs::EigenTrajectoryPoint* reference) const
{
  assert(reference != nullptr);
  *reference = command_trajectory_;
  return true;
}

}
