/*
 * aug_cov_test.cpp
 *
 *  Created on: Sep 11, 2012
 *      Author: acmarkus
 */

#include <mav_planning_utils/augmented_cost.h>
#include <string>
#include <iostream>
#include <stdlib.h>

using namespace Eigen;
using namespace std;

void printResult(const string& text, const AugmentedCost<2>& c1,
                 const AugmentedCost<2>& c2) {
  double eps;
  bool cmp = c1.comparePartial(c2, 0, &eps);
  cout << text << " c1 dominates: " << cmp << " (" << c1.getMetric() << " "
       << c2.getMetric() << " " << eps << ")" << endl;

  bool cmp_eps = c1.comparePartial(c2, 0.1, &eps);
  cout << text << " c1 dominates eps: " << cmp_eps << " (" << c1.getMetric()
       << " " << c2.getMetric() << " " << eps << ")" << endl;
}

void testComparision(const Matrix2d& first, const Matrix2d& second,
                     const Vector2d& ref) {
  AugmentedCost<2> c1(first, 0, ref);
  AugmentedCost<2> c2;
  c2.setReference(ref);
  c2.setSigma(second);
  c2.setCost(0);

  cout << "\n\ntesting matrices:\n" << first << endl << second << endl;

  c1.computeMetricTrace();
  c2.computeMetricTrace();
  printResult("Trace", c1, c2);

  c1.computeMetricDeterminant();
  c2.computeMetricDeterminant();
  printResult("Det", c1, c2);

  c1.computeMetricFoerstner();
  c2.computeMetricFoerstner();
  printResult("Foerstner", c1, c2);

  c1.computeMetricKLDivergence();
  c2.computeMetricKLDivergence();
  printResult("KV Divergence", c1, c2);

  c1.computeMetricSmallestEigenValue();
  c2.computeMetricSmallestEigenValue();
  printResult("Eig", c1, c2);
}

int main(int argc, char** argv) {
  Matrix2d sr, lr, sf, lf;  // {small|large}{round|flat}
  Vector2d ref(Vector2d::Ones() * 1e-9);

  sr << 1, 0, 0, 1;
  sr *= 1e-6;
  lr = sr * 2;

  sf << 10, 0, 0, 0.01;
  sf *= 1e-6;
  lf = sf * 2;

  //  testComparision(sr, lr, ref);
  //  testComparision(sf, lf, ref);
  //  testComparision(sr, lf, ref);

  if (argc == 3) {
    Matrix2d in;
    in << atof(argv[1]), 0, 0, atof(argv[2]);
    testComparision(sr, in, ref);
  }
}
