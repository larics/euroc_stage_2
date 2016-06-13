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

#include <algorithm>

#include <gtest/gtest.h>

#include <mav_planning_utils/polynomial.h>

#include "polynomial_testdata.h"

using namespace mav_planning_utils;

const int N = 10;
typedef Polynomial<N> P;

bool compareComplexRoots(const Eigen::VectorXcd& roots1,
                         const Eigen::VectorXcd& roots2, double tol = 1e-9) {
  if (roots1.size() != roots2.size()) return false;

  size_t n_roots = roots1.size();

  std::vector<std::complex<double> > vr1, vr2;

  for (size_t i = 0; i < n_roots; ++i) {
    vr1.push_back(roots1[i]);
    vr2.push_back(roots2[i]);
  }

  struct Cmp {
    bool operator()(const std::complex<double>& lhs,
                    const std::complex<double>& rhs) const {
      std::pair<double, double> c1(real(lhs), imag(lhs));
      std::pair<double, double> c2(real(rhs), imag(rhs));
      return c1 < c2;
    }
  } cmp;

  std::sort(vr1.begin(), vr1.end(), cmp);
  std::sort(vr2.begin(), vr2.end(), cmp);

  bool success = true;
  for (size_t i = 0; i < n_roots; ++i) {
    if (std::abs(real(vr1[i]) - real(vr2[i])) < tol &&
        std::abs(imag(vr1[i]) - imag(vr2[i])) > tol) {
      success = false;
      break;
    }
  }

  if (!success) {
    std::cout << "[FAILURE]: roots do not match: \n";

    std::cout << "    TRUE:   ";
    for (const std::complex<double>& c : vr1) std::cout << c;
    std::cout << std::endl;

    std::cout << "    ACTUAL: ";
    for (const std::complex<double>& c : vr2) std::cout << c;
    std::cout << std::endl;

    std::cout << "    DIFF:   ";
    for (size_t i = 0; i < n_roots; ++i) {
      std::cout << vr1[i] - vr2[i];
    }
    std::cout << std::endl;
  }

  return success;
}

TEST(MavPlanningUtils, Polynomial_compute_roots) {
  for (int i = 0; i < coefficients_simple_rows; ++i) {
    Polynomial<N_simple>::VectorR coefficients(coefficients_simple[i]);
    Polynomial<N_simple> p(coefficients);
    Eigen::VectorXcd roots = p.computeRoots();

    Eigen::VectorXcd true_roots(N_simple - 1);
    for (int j = 0; j < N_simple - 1; ++j) {
      true_roots[j] = std::complex<double>(roots_simple_real[i][j],
                                           roots_simple_imag[i][j]);
    }

    EXPECT_TRUE(compareComplexRoots(true_roots, roots));
  }
}

TEST(MavPlanningUtils, Polynomial_compute_roots_high_order) {
  for (int i = 0; i < coefficients_v_magnitude_rows; ++i) {
    Polynomial<N_v_magnitude>::VectorR coefficients(
        coefficients_v_magnitude[i]);
    Polynomial<N_v_magnitude> p(coefficients);
    Eigen::VectorXcd roots = p.computeRoots();

    Eigen::VectorXcd true_roots(N_v_magnitude - 1);
    for (int j = 0; j < N_v_magnitude - 1; ++j) {
      true_roots[j] = std::complex<double>(roots_v_magnitude_real[i][j],
                                           roots_v_magnitude_imag[i][j]);
    }

    EXPECT_TRUE(compareComplexRoots(true_roots, roots, 1.0e-6));
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
