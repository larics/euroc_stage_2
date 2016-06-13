/*
 * redheffer_star_product.h
 *
 *  Created on: May 11, 2012
 *      Author: acmarkus
 */

#ifndef REDHEFFER_STAR_PRODUCT_H_
#define REDHEFFER_STAR_PRODUCT_H_

#include <Eigen/Dense>
#include <Eigen/SVD>

template <int n, class T = double>
class ScatteringMatrixC {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<T, n, n> F;
  Eigen::Matrix<T, n, n> Q;

  ScatteringMatrixC() {}
  ScatteringMatrixC(const Eigen::Matrix<T, n, n> &_F,
                    const Eigen::Matrix<T, n, n> &_Q)
      : F(_F), Q(_Q) {}
};

template <int n, class T = double>
class ScatteringMatrixM {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<T, n, n> M;

  ScatteringMatrixM() {}
  ScatteringMatrixM(const Eigen::Matrix<T, n, n> &_M) : M(_M) {}
  template <int n_m>
  ScatteringMatrixM(const Eigen::Matrix<T, n_m, n> &H,
                    const Eigen::Matrix<T, n_m, n_m> &R) {
    M = H.transpose() * R.inverse() * H;
  }
};

template <int n, class T = double>
class ScatteringMatrix {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<T, n, n> A;
  Eigen::Matrix<T, n, n> B;
  Eigen::Matrix<T, n, n> C;
  Eigen::Matrix<T, n, n> D;

  ScatteringMatrix() {}
  ScatteringMatrix(const Eigen::Matrix<T, n, n> &_A,
                   const Eigen::Matrix<T, n, n> &_B,
                   const Eigen::Matrix<T, n, n> &_C,
                   const Eigen::Matrix<T, n, n> &_D)
      : A(_A), B(_B), C(_C), D(_D) {}
};

template <int n, class T>
ScatteringMatrix<n, T> operator*(const ScatteringMatrix<n, T> &lhs,
                                 const ScatteringMatrixC<n, T> &rhs) {
  ScatteringMatrix<n, T> res;
  res.A = rhs.F * lhs.A;
  res.B = rhs.Q + rhs.F * lhs.B * rhs.F.transpose();
  res.C = lhs.C;
  res.D = lhs.D * rhs.F.transpose();
  return res;
}

template <int n, class T>
ScatteringMatrix<n, T> operator*(const ScatteringMatrix<n, T> &lhs,
                                 const ScatteringMatrixM<n, T> &rhs) {
  ScatteringMatrix<n, T> res;

  Eigen::Matrix<T, n, n> iBM;
  iBM = Eigen::Matrix<T, n, n>::Identity() + lhs.B * rhs.M;
  iBM = iBM.inverse();

  Eigen::Matrix<T, n, n> iMB;
  iMB = Eigen::Matrix<T, n, n>::Identity() + rhs.M * lhs.B;
  iMB = iMB.inverse();

  res.A = iBM * lhs.A;
  res.B = iBM * lhs.B;
  res.D = lhs.D * iMB;
  res.C = lhs.C - lhs.D * iMB * rhs.M * lhs.A;
  return res;
}

#endif /* REDHEFFER_STAR_PRODUCT_H_ */
