/*
 * augmented_cost.h
 *
 *  Created on: Sep 11, 2012
 *      Author: acmarkus
 */

#ifndef AUGMENTED_COST_H_
#define AUGMENTED_COST_H_

#include <Eigen/Dense>
#include <iostream>

template <int N, class _scalar = double>
class AugmentedCost {
 public:
  typedef _scalar Scalar;
  typedef Eigen::Matrix<Scalar, N, 1> X;
  typedef Eigen::Matrix<Scalar, N, N> StateCov;

  enum MetricType {
    SMALLEST_EIGENVALUE,
    FOERSTNER,
    KL_DIVERGENCE,
    TRACE,
    DETERMINANT,
    NONE = -1
  };

 private:
  StateCov sigma_;
  X ref_;  // only a diagonal matrix is used as reference
  Scalar cost_;
  Scalar cov_metric_;
  X eigenvalues_;

  // convenience variables
  X ref_inv_;
  Scalar ref_det_;

  MetricType metric_type_;

  void setupInternals() {
    ref_det_ = ref_.prod();
    ref_inv_ = ref_.cwiseInverse();
    assert(ref_det_ > 0);
    cov_metric_ = 1e12;
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AugmentedCost() : cost_(0), metric_type_(NONE) {}

  template <class DerivedSigma, class DerivedRef>
  AugmentedCost(const Eigen::MatrixBase<DerivedSigma> &sigma, Scalar cost,
                const Eigen::MatrixBase<DerivedRef> &ref)
      : sigma_(sigma), ref_(ref), cost_(cost), metric_type_(NONE) {
    setupInternals();
  }

  AugmentedCost(Scalar sigma, Scalar cost, Scalar ref = 1e-9)
      : cost_(cost), metric_type_(NONE) {
    sigma_ = sigma * StateCov::Identity();
    ref_.setConstant(ref);

    setupInternals();
  }

  void setCost(Scalar cost) { cost_ = cost; }

  void addCost(Scalar cost) { cost_ += cost; }

  void setReference(Scalar ref) {
    ref_.setConstant(ref);
    setupInternals();
  }

  template <class DerivedSigma>
  void setSigma(const Eigen::MatrixBase<DerivedSigma> &sigma) {
    sigma_ = sigma;
  }

  template <class Derived>
  void setReference(const Eigen::MatrixBase<Derived> &ref) {
    ref_ = ref;
    setupInternals();
  }

  bool computeMetricFoerstner() {
    Eigen::GeneralizedSelfAdjointEigenSolver<StateCov> solver(
        sigma_, ref_.asDiagonal());
    X eig = solver.eigenvalues();

    Scalar eig_ln;
    cov_metric_ = 0;
    for (int i = 0; i < N; i++) {
      eig_ln = log(eig[i]);
      cov_metric_ += eig_ln * eig_ln;
    }

    cov_metric_ = sqrt(cov_metric_);
    metric_type_ = FOERSTNER;
    return true;
  }

  bool computeMetricKLDivergence() {
    Scalar det = sigma_.determinant();
    if (det <= 0) {
      sigma_ = (sigma_ + sigma_.transpose()) * 0.5;
      det = sigma_.determinant() + 1e-42;  // TODO: Fix this hack
    }
    if (ref_det_ <= 0 || det < 0) {
      std::cout << "[Error:] ref_det or det < 0 : " << ref_det_ << "/" << det
                << std::endl;
      return false;
    }

    cov_metric_ = 0.5 * ((ref_inv_.asDiagonal() * sigma_).trace() -
                         log(det / ref_det_) - static_cast<Scalar>(N));
    metric_type_ = KL_DIVERGENCE;
    return true;
  }

  bool computeMetricSmallestEigenValue() {
    eigenvalues_ =
        Eigen::SelfAdjointEigenSolver<StateCov>(sigma_).eigenvalues();
    metric_type_ = SMALLEST_EIGENVALUE;
    return true;
  }

  bool computeMetricTrace() {
    cov_metric_ = sigma_.trace();
    metric_type_ = TRACE;
    return true;
  }

  bool computeMetricDeterminant() {
    cov_metric_ = sigma_.determinant();
    metric_type_ = DETERMINANT;
    return true;
  }

  bool computeMetric(int metric = TRACE) {
    // SMALLEST_EIGENVALUE, FOERSTNER, KV_DIVERGENCE, TRACE, DETERMINANT
    switch (metric) {
      case SMALLEST_EIGENVALUE:
        return computeMetricSmallestEigenValue();
      case FOERSTNER:
        return computeMetricFoerstner();
      case KL_DIVERGENCE:
        return computeMetricKLDivergence();
      case TRACE:
        return computeMetricTrace();
      case DETERMINANT:
        return computeMetricDeterminant();
    }
  }

  /*
   * returns true if this dominates (i.e. is less than) other
   */
  bool comparePartial(const AugmentedCost &other, Scalar epsilon = 0,
                      Scalar *epsilon_out = NULL) const {
    assert(other.metric_type_ == metric_type_ && metric_type_ != NONE &&
           other.metric_type_ != NONE);

    if (epsilon == 0) {
      epsilon = 1e-6;
    }

    // cost
    Scalar cost_diff = other.getCost() - cost_;
    if (cost_diff < 0) return false;

    // covariance
    Scalar diff;

    if (metric_type_ == SMALLEST_EIGENVALUE) {
      diff = (other.eigenvalues_ - eigenvalues_).minCoeff();
      epsilon = eigenvalues_.sum() * epsilon;
    } else {
      diff = other.cov_metric_ - cov_metric_;
      epsilon = epsilon * cov_metric_;
    }

    if (epsilon_out) *epsilon_out = epsilon;

    if (diff < -epsilon) return false;

    return true;
  }

  /*
   * returns true if this has lower cost
   */
  bool compareTotal(const AugmentedCost &other) const {
    return cost_ < other.cost_;
  }

  Scalar getCost() const { return cost_; }

  StateCov getSigma() const { return sigma_; }

  template <class Derived>
  void getSigma(const Eigen::MatrixBase<Derived> &sigma) const {
    static_cast<Eigen::MatrixBase<Derived> &>(sigma) = sigma_;
  }

  Scalar getMetric() const { return cov_metric_; }
};

#endif /* AUGMENTED_COST_H_ */
