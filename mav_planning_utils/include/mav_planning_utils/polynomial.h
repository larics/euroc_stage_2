/*

 Copyright (c) 2013, Markus Achtelik, ASL, ETH Zurich, Switzerland
 You can contact the author at <markus dot achtelik at mavt dot ethz dot ch>

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#ifndef POLYNOMIAL_H_
#define POLYNOMIAL_H_

#include <iostream>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/SVD>
#include <glog/logging.h>

#include <mav_planning_utils/rpoly.h>

namespace mav_planning_utils {
/**
 * \brief Implementation of polynomials of order _N-1. Order must be known at
 * compile time.
 * Polynomial coefficients are stored with increasing powers, i.e. \f$c_0 +
 * c_1*t ... c_{N-1} * t^{N-1}\f$
 * \tparam _N Number of coefficients of the polynomial.
 * \tparam ScalarT Floating point type of the scalar.
 */
template <int _N, class ScalarT = double>
class Polynomial {
 public:
  // parrot back template arguments
  typedef ScalarT Scalar;
  const static int N = _N;
  const static int DEG = N - 1;

  typedef Eigen::Matrix<Scalar, 1, N> VectorR;
  typedef Eigen::Matrix<Scalar, N, 1> VectorV;
  typedef Eigen::Matrix<Scalar, N, N> MatrixSq;
  typedef std::complex<Scalar> Complex;
  typedef Eigen::VectorXcd RootVector;

 private:
  VectorR coefficients_;

  template <int I>
  struct Int2Type {
    enum { value = I };
  };

  /// Overload of implementation for getCoefficients() in case derivative is 0.
  Eigen::Matrix<Scalar, 1, N> getCoefficientsImpl(Int2Type<0>) const {
    return coefficients_;
  }

  /// Generic implementation for getCoefficients().
  template <int derivative>
  Eigen::Matrix<Scalar, 1, N - derivative> getCoefficientsImpl(
      Int2Type<derivative>) const {
    static_assert(derivative <= N, "derivative needs to be <= N");

    return coefficients_.template tail<N - derivative>().cwiseProduct(
        base_coefficients_.row(derivative).template tail<N - derivative>());
  }

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static MatrixSq base_coefficients_;

  Polynomial() {}

  template <class Derived>
  Polynomial(const Eigen::MatrixBase<Derived> &coeffs)
      : coefficients_(coeffs) {}

  /**
   * \brief sets up the internal representation from coeffs
   * coefficients are stored in increasing order with the power of t, i.e. c1 +
   * c2*t + c3*t^2 ==> coeffs = [c1 c2 c3]
   */
  template <class Derived>
  void setCoefficients(const Eigen::MatrixBase<Derived> &coeffs) {
    coefficients_ = coeffs;
  }

  /**
   * \brief Returns the coefficients for the specified derivative of the
   * polynomial.
   */
  Eigen::Matrix<ScalarT, 1, Eigen::Dynamic> getCoefficients(
      int derivative = 0) const {
    assert(derivative <= N);
    if (derivative == 0)
      return coefficients_;
    else
      return coefficients_.tail(N - derivative)
          .cwiseProduct(
              base_coefficients_.row(derivative).tail(N - derivative));
  }

  /**
   * \brief Returns the coefficients for the specified derivative of the
   * polynomial. Static version, if derivative is known at compile time.
   */
  template <int derivative>
  Eigen::Matrix<Scalar, 1, N - derivative> getCoefficients() const {
    Int2Type<derivative> derivative_type;
    return getCoefficientsImpl(derivative_type);
  }

  /**
   * \brief Computes the Jacobian of the integral over the squared derivative
   * \param[out] C Jacobian matrix to write into. If C is dynamic, the correct
   * size has to be set.
   * \param[in] t time of evaluation
   * \param[in] derivative used to compute the cost
   */
  template <class Derived>
  static void quadraticCostJacobian(const Eigen::MatrixBase<Derived> &C,
                                    Scalar t, int derivative) {
    CHECK_LT(derivative, N);
    if (Derived::RowsAtCompileTime == Eigen::Dynamic ||
        Derived::ColsAtCompileTime == Eigen::Dynamic) {
      CHECK_EQ(C.rows(), N);
      CHECK_EQ(C.cols(), N);
    } else {
      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, N, N);
    }

    Eigen::MatrixBase<Derived> &_C =
        const_cast<Eigen::MatrixBase<Derived> &>(C);
    _C.setZero();

    for (int col = 0; col < N - derivative; col++) {
      for (int row = 0; row < N - derivative; row++) {
        Scalar exp = (DEG - derivative) * 2 + 1 - row - col;

        _C(DEG - row, DEG - col) = base_coefficients_(derivative, N - 1 - row) *
                                   base_coefficients_(derivative, N - 1 - col) *
                                   pow(t, exp) * 2 / exp;
      }
    }
  }

  /**
   * \brief convenience method to compute the Jacobian of the quadratic cost
   * \sa static void quadraticCostJacobian(const Eigen::MatrixBase<Derived> & C,
   * Scalar t, int derivative)
   */
  static MatrixSq quadraticCostJacobian(Scalar t, int derivative) {
    MatrixSq C;
    quadraticCostJacobian(C, t, derivative);
    return C;
  }

  /**
   * \brief Computes the base coefficients with the according powers of t, as
   * e.g. needed for computation of (in)equality constraints
   * \param[out] coeffs vector to write the coefficients to
   * \param[in] derivative of the polynomial for which the coefficients have to
   * be computed
   * \param[in] t time of evaluation
   */
  template <class Derived>
  static void baseCoeffsWithTime(const Eigen::MatrixBase<Derived> &coeffs,
                                 int derivative, Scalar t) {
    CHECK_LT(derivative, N);
    CHECK_GE(derivative, 0);
    Eigen::MatrixBase<Derived> &c =
        const_cast<Eigen::MatrixBase<Derived> &>(coeffs);

    c.setZero();
    // first coefficient doesn't get multiplied
    c[derivative] = base_coefficients_(derivative, derivative);

    if (t == 0) return;

    Scalar _t = t;
    // now multiply increasing power of t towards the right
    for (int j = derivative + 1; j < N; j++) {
      c[j] = base_coefficients_(derivative, j) * _t;
      _t = _t * t;
    }
  }

  /**
   * \brief Convenience method to compute the base coefficents with time
   * \sa static void baseCoeffsWithTime(const Eigen::MatrixBase<Derived> &
   * coeffs, int derivative, Scalar t)
   */
  static Eigen::Matrix<Scalar, 1, N> baseCoeffsWithTime(int derivative,
                                                        Scalar t) {
    Eigen::Matrix<Scalar, N, 1> c;
    baseCoeffsWithTime(c, derivative, t);
    return c;
  }

  /// evaluates the polynomial at time t and writes the result to result
  template <class Derived>
  void evaluate(const Eigen::MatrixBase<Derived> &result, Scalar t) const {
    CHECK_LE(result.rows(),
             N);  // runtime assertion, because a dynamic one is fine as well
    EIGEN_STATIC_ASSERT_VECTOR_ONLY(Derived);
    const int max_deg = result.size();

    Eigen::MatrixBase<Derived> &_result =
        const_cast<Eigen::MatrixBase<Derived> &>(result);

    for (int i = 0; i < max_deg; i++) {
      const int tmp = N - 1;
      const VectorR row = base_coefficients_.row(i);
      Scalar acc = row[tmp] * coefficients_[tmp];
      for (int j = tmp - 1; j >= i; --j) {
        acc *= t;
        acc += row[j] * coefficients_[j];  // coefficients_(i, j);
      }
      _result[i] = acc;
    }
  }

  /// evaluates the specified derivative of the polynomial at time t and writes
  /// the result to result
  void evaluate(Scalar &result, Scalar t, int derivative) const {
    CHECK_LT(derivative, N);
    const int tmp = N - 1;
    const VectorR row = base_coefficients_.row(derivative);
    result = row[tmp] * coefficients_[tmp];
    for (int j = tmp - 1; j >= derivative; --j) {
      result *= t;
      result += row[j] * coefficients_[j];
    }
  }

  /// evaluates the specified derivative of the polynomial at time t and returns
  /// the result
  Scalar evaluate(Scalar t, int derivative) const {
    Scalar res;
    evaluate(res, t, derivative);
    return res;
  }

  /// evaluates the polynomial at time t and returns the result
  template <int max_deg>
  inline Eigen::Matrix<Scalar, max_deg, 1> evaluate(Scalar t) const {
    Eigen::Matrix<Scalar, max_deg, 1> result;
    evaluate(result, t);
    return result;
  }

  /// evaluates the polynomial at times in t and writes the result for each time
  /// into the corresponding column of result
  template <int max_deg, int n_samples>
  void evaluate(const Eigen::Matrix<Scalar, 1, n_samples> &t,
                Eigen::Matrix<Scalar, max_deg, n_samples> result) const {
    Eigen::Matrix<Scalar, max_deg, 1> _result;
    for (int i = 0; i < n_samples; i++) {
      evaluate(t[i], _result);
      result.col(i) = result;
    }
  }

  /**
   * \brief Computes the complex roots of the polynomial.
   * Only for the polynomial itself, not for its derivatives.
   */
  Eigen::VectorXcd computeRoots() const {
    //      Companion matrix method , see
    //      http://en.wikipedia.org/wiki/Companion_matrix.
    //      Works, but is not very stable for high condition numbers. Could be
    //      eigen's eigensolver.
    //      However, would not need the dependency to rpoly.
    //      const size_t nc = N - 1;
    //      typedef Eigen::Matrix<Scalar, nc, nc> CompanionMatrix;
    //      CompanionMatrix companion;
    //      companion.template row(0).setZero();
    //      companion.template block<nc - 1, nc - 1>(1, 0).setIdentity();
    //      companion.template col(nc - 1) = - coefficients_.template head<nc>()
    //      / coefficients_[N - 1];
    //
    //      Eigen::EigenSolver<CompanionMatrix> es(companion, false);
    //      return es.eigenvalues();

    return findRootsJenkinsTraub(coefficients_);
  }
};

// static member initialization

/// computes the basis coefficients of the derivatives of the polynomial
template <int N, class Scalar>
typename Polynomial<N, Scalar>::MatrixSq computeBaseCoefficients() {
  typename Polynomial<N, Scalar>::MatrixSq base_coefficients;

  base_coefficients.setZero();
  base_coefficients.row(0) = Polynomial<N, Scalar>::VectorR::Ones();

  const int DEG = Polynomial<N, Scalar>::DEG;
  int order = DEG;
  for (int n = 1; n < N; n++) {
    for (int i = DEG - order; i < N; i++) {
      base_coefficients(n, i) = (order - DEG + i) * base_coefficients(n - 1, i);
    }
    order--;
  }
  return base_coefficients;
}

template <int N, class Scalar>
typename Polynomial<N, Scalar>::MatrixSq Polynomial<
    N, Scalar>::base_coefficients_ = computeBaseCoefficients<N, Scalar>();

}  // end namespace
#endif /* MINIMUM_SNAP_TRAJECTORY_H_ */
