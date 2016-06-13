/*

 Copyright (c) 2014, Markus Achtelik, ASL, ETH Zurich, Switzerland
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

#include <gtest/gtest.h>

#include <mav_planning_utils/path_planning.h>
#include <mav_planning_utils/yaml_io.h>
#include <mav_planning_utils/testing_predicates.h>

#include <limits>

using namespace mav_planning_utils::path_planning;

TEST(MavPlanningUtils, PathPlanning_VertexComparisons) {
  Vertex4D v1, v2, v3, v4;

  v1.time_to_next = 1.5;
  v1.derivative_to_optimize = 3;
  v1.addConstraint(0, Vertex4D::ConstraintValueT::Random());
  v1.addConstraint(1, Vertex4D::ConstraintValueT::Random());
  v1.addConstraint(2, Vertex4D::ConstraintValueT::Random());

  v2 = v1;
  v3 = v1;

  double tol = 1e-6;
  v3.addConstraint(
      0, v1.getConstraint(0) + Vertex4D::ConstraintValueT::Constant(tol * 0.9));
  v4.addConstraint(
      0, v1.getConstraint(0) + Vertex4D::ConstraintValueT::Constant(tol * 1.1));

  EXPECT_TRUE(v1.isEqualTol(v2, 0));
  EXPECT_TRUE(v1.isEqualTol(v3, tol));
  EXPECT_FALSE(v1.isEqualTol(v4, tol));

  v2.addConstraint(4, Vertex4D::ConstraintValueT::Random());
  EXPECT_FALSE(v1.isEqualTol(v2, 0));
}

TEST(MavPlanningUtils, YamlIo_VertexParsing) {
  std::stringstream yaml;
  yaml << "vertex:\n";
  yaml << "  time_to_next: 1.5\n";
  yaml << "  derivative_to_optimize: s\n";  // should be 4 later
  yaml << "  constraints:\n";
  yaml << "    - type: p\n";
  yaml << "      value: [0, 0, 0, 0]\n";
  yaml << "    - type: v\n";
  yaml << "      value: [1, 1, 1, 1]\n";
  yaml << "    - type: a\n";
  yaml << "      value: [2, 2, 2, 2]\n";

  YAML::Parser parser(yaml);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  Vertex4D v;
  doc >> v;

  EXPECT_DOUBLE_EQ(1.5, v.time_to_next);
  EXPECT_EQ(v.derivative_to_optimize, 4);
  EXPECT_NEAR_EIGEN(v.getConstraint(mav_planning_utils::DerivativesP::p),
                    Vertex4D::ConstraintValueT::Constant(0), 0);
  EXPECT_NEAR_EIGEN(v.getConstraint(mav_planning_utils::DerivativesP::v),
                    Vertex4D::ConstraintValueT::Constant(1), 0);
  EXPECT_NEAR_EIGEN(v.getConstraint(mav_planning_utils::DerivativesP::a),
                    Vertex4D::ConstraintValueT::Constant(2), 0);
}

TEST(MavPlanningUtils, YamlIo_OptionalKeys) {
  std::stringstream yaml;
  yaml << "waypoints:\n";
  yaml << "  v_max: 2.5\n";
  yaml << "  a_max: 3\n";
  yaml << "  vertices:\n";

  YAML::Parser parser(yaml);
  YAML::Node doc;
  parser.GetNextDocument(doc);

  mav_planning_utils::path_planning::Waypoints4D wpts;
  doc >> wpts;

  EXPECT_DOUBLE_EQ(2.5, wpts.v_max);
  EXPECT_DOUBLE_EQ(3, wpts.a_max);
  EXPECT_DOUBLE_EQ(0, wpts.j_max);
  EXPECT_DOUBLE_EQ(0, wpts.s_max);
  EXPECT_DOUBLE_EQ(0, wpts.yaw_dot_max);
}

TEST(MavPlanningUtils, YamlIo_EmitParse) {
  Vertex4D v1, v2;

  v1.time_to_next = 1.5;
  v1.derivative_to_optimize = 3;
  v1.addConstraint(0, Vertex4D::ConstraintValueT::Random());
  v1.addConstraint(1, Vertex4D::ConstraintValueT::Random());
  v1.addConstraint(2, Vertex4D::ConstraintValueT::Random());

  v2 = v1;
  v2.addConstraint(1, Vertex4D::ConstraintValueT::Random());
  v2.addConstraint(3, Vertex4D::ConstraintValueT::Random());

  Waypoints4D wp_in(1, 2, 3, 4, 5), wp_out;
  wp_in.waypoints.push_back(v1);
  wp_in.waypoints.push_back(v2);

  saveWaypoints("test.yaml", wp_in);

  loadWaypoints("test.yaml", wp_out);

  EXPECT_TRUE(
      wp_in.isEqualTol(wp_out, 4 * std::numeric_limits<double>::epsilon()));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
