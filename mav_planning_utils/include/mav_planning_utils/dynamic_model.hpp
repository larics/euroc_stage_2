/*
 * dynamic_model.hpp
 *
 *  Created on: Jan 26, 2012
 *      Author: acmarkus
 */

#ifndef DYNAMIC_MODEL_HPP_
#define DYNAMIC_MODEL_HPP_

#include <Eigen/Dense>
#include <Eigen/LU>
#include <iostream>

namespace mav_planning_utils {

/**
 * \class DynamicModel Nth order closed loop dynamic model.
 * \tparam N order of the model
 */
template <int N = 2>
class DynamicModel {
 public:
  typedef Eigen::Matrix<double, N, N> A;
  typedef Eigen::Matrix<double, N, 1> B;
  typedef Eigen::Matrix<double, 1, N> K;
  typedef Eigen::Matrix<double, N, 1> x;

 private:
  A A_;
  K K_;
  x x_;
  double x_c_old_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  DynamicModel() : A_(A::Zero()), K_(K::Zero()), x_(x::Zero()), x_c_old_(0) {
    A_.template diagonal<1>() = Eigen::Matrix<double, N - 1, 1>::Ones();
  }

  /**
   * \brief Advances the model by one step.
   * Additionally, a maximum velocity can be set.
   * \param[in] x_c Commanded goal state to drag the model towards
   * \param[in] dt time step
   * \param[in] v_max maximum velocity
   * \param[out] the computed control input to the model
   * \return the new state
   */
  inline x step(const double &x_c, const double &dt, const double &v_max = 10,
                double *u = NULL) {
    Eigen::Matrix<double, 1, 1> _u;

    double v_tmp = (x_c - x_c_old_) / dt;
    // TODO: make that more efficient
    if (v_tmp > v_max)
      x_c_old_ = x_c_old_ + dt * v_max;
    else if (v_tmp < -v_max)
      x_c_old_ = x_c_old_ - dt * v_max;
    else
      x_c_old_ = x_c;

    //      std::cout << v_tmp << " "<< x_c_old_ << std::endl;

    _u = -K_ * x_;
    _u += K_.template head<1>() * x_c_old_;  // u = -K*(x - [x_c  0 ... 0 ])

    x_ = x_ + A_ * x_ * dt;
    x_.template tail<1>() -=
        _u * dt;  // x_k+1 = x + (A*x + B*u)*dt; B=[0 0 ... -1]'

    if (u) *u = _u[0];

    return x_;
  }

  /**
   * \brief Resets the internal state to zeros or _x
   */
  void reset(const x &_x = x::Zero()) {
    x_c_old_ = _x[0];
    x_ = _x;
  }

  /**
   * \brief Computes the feedback matrix K based on the given poles
   */
  void placePoles(const Eigen::Matrix<double, 1, N> &poles) {
    A a;
    B b;

    Eigen::Matrix2d test;
    test.block<1, 1>(0, 0);

    a.setOnes();

    for (int i = 0; i < N; i++) {
      for (int j = 0; j < i; j++) {
        a.template block<N, 1>(0, i) =
            a.template block<N, 1>(0, i).cwiseProduct(poles.transpose());
      }
      b(i) = pow(poles(i), N);
    }
    K_ = (a.inverse() * b).transpose();

    //      std::cout<<"A:"<<a<<std::endl<<"K:"<<K_<<std::endl;
  }
};

};  // end namespace

#endif /* DYNAMIC_MODEL_HPP_ */
