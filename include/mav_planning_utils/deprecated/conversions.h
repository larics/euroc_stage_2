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

#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <mav_planning_utils/mav_state.h>
#include <mav_planning_utils/motion4D.h>
#include <mav_planning_utils/motion_defines.h>
#include <mav_planning_utils/sincos.h>

namespace mav_planning_utils {

template <int n_pos, int n_yaw, class T>
inline void motion4dToMulticopter(MavState& mc_state,
                                  const Motion4D<n_pos, n_yaw, T>& m4d,
                                  double magnitude_of_gravity = 9.81) {
  EIGEN_STATIC_ASSERT(n_pos == 5,
                      THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)
  EIGEN_STATIC_ASSERT(n_yaw >= 2,
                      THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

  // compute motion dependent states
  applyMulticopterModel(
      m4d.getStateP(DerivativesP::a), m4d.getStateP(DerivativesP::j),
      m4d.yaw[DerivativesO::o], m4d.yaw[DerivativesO::w], mc_state.q,
      mc_state.a_b, mc_state.w_b, magnitude_of_gravity);

  // copy remaining states
  mc_state.p = m4d.getStateP(DerivativesP::p);
  mc_state.v = m4d.getStateP(DerivativesP::v);
  mc_state.a = m4d.getStateP(DerivativesP::a);
  mc_state.j = m4d.getStateP(DerivativesP::j);
  mc_state.s = m4d.getStateP(DerivativesP::s);
}

template <int n_pos, int n_yaw, class T>
inline void multicopterToMotion4d(Motion4D<n_pos, n_yaw, T>& m4d,
                                  const MavState& mc_state) {
  EIGEN_STATIC_ASSERT(n_pos == 5,
                      THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)
  EIGEN_STATIC_ASSERT(n_yaw == 3,
                      THIS_METHOD_IS_ONLY_FOR_VECTORS_OF_A_SPECIFIC_SIZE)

  m4d.setStateP(DerivativesP::p, mc_state.p);
  m4d.setStateP(DerivativesP::v, mc_state.v);
  m4d.setStateP(DerivativesP::a, mc_state.a);
  m4d.setStateP(DerivativesP::j, mc_state.j);
  m4d.setStateP(DerivativesP::s, mc_state.s);
}
}

#endif /* CONVERSIONS_H_ */
