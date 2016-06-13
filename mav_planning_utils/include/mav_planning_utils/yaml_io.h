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

#ifndef YAML_IO_H_
#define YAML_IO_H_

#include <yaml-cpp/yaml.h>
#include <mav_planning_utils/path_planning.h>

#include <iostream>
#include <fstream>

namespace YAML {
template <int D>
struct convert<mav_planning_utils::path_planning::Vertex<D> > {
  // TODO(burrimi): we dont seem to need it, but we should implement this at
  // some point.
  //      static Node encode(const mav_planning_utils::path_planning::Vertex<D>
  //      & rhs) {
  //        Node node;
  //        return node;
  //      }
  static bool decode(const Node& node,
                     mav_planning_utils::path_planning::Vertex<D>& rhs) {
    const YAML::Node& v_node = node["vertex"];

    rhs.time_to_next = v_node["time_to_next"].as<double>();
    std::string tmp;
    tmp = v_node["derivative_to_optimize"].as<std::string>();
    rhs.derivative_to_optimize = mav_planning_utils::DerivativesP::toInt(tmp);

    const YAML::Node& constraints = v_node["constraints"];
    for (size_t i = 0; i < constraints.size(); ++i) {
      std::string tmp_val;
      tmp_val = constraints[i]["type"].as<std::string>();
      const YAML::Node& value_y = constraints[i]["value"];

      if (value_y.size() != D) {
        throw YAML::ParserException(YAML::Mark::null_mark(),
                                    "constraint sizes do not match");
      }

      typename mav_planning_utils::path_planning::Vertex<D>::ConstraintValueT
          value;
      int j = 0;
      for (YAML::const_iterator it = value_y.begin(); it != value_y.end();
           ++it) {
        value[j] = it->as<double>();
        ++j;
      }
      int type = mav_planning_utils::DerivativesP::toInt(tmp_val);
      if (type == mav_planning_utils::DerivativesP::none)
        throw YAML::ParserException(
            YAML::Mark::null_mark(),
            "couldn't de-serialize value for type of constraint: " + tmp_val);
      rhs.addConstraint(type, value);
    }
    return true;
  }
};
}

namespace mav_planning_utils {
namespace path_planning {

/// Reads a waypoint list from yaml node and writes to
/// mav_planning_utils::path_planning::Waypoints structure.
template <int D>
void operator>>(const YAML::Node& node,
                mav_planning_utils::path_planning::Waypoints<D>& w) {
  const YAML::Node& wp_node = node["waypoints"];

  struct OptKey {
    static double fromYaml(const YAML::Node& node,
                           const std::string& key_name) {
      if (YAML::Node parameter = node[key_name]) {
        double tmp;
        tmp = parameter.as<double>();
        return tmp;
      } else
        return 0;
    }
  };

  w.v_max = OptKey::fromYaml(wp_node, "v_max");
  w.a_max = OptKey::fromYaml(wp_node, "a_max");
  w.j_max = OptKey::fromYaml(wp_node, "j_max");
  w.s_max = OptKey::fromYaml(wp_node, "s_max");
  w.yaw_dot_max = OptKey::fromYaml(wp_node, "yaw_dot_max");

  for (YAML::const_iterator it = wp_node["vertices"].begin();
       it != wp_node["vertices"].end(); ++it) {
    w.waypoints.push_back(
        it->as<mav_planning_utils::path_planning::Vertex<D> >());
  }
}

// Writes a vertex structure to a yaml node.
template <int D>
YAML::Emitter& operator<<(
    YAML::Emitter& out,
    const mav_planning_utils::path_planning::Waypoints<D>& w) {
  typedef mav_planning_utils::path_planning::Waypoints<D> Waypoints;

  out << YAML::BeginMap << YAML::Key << "waypoints" << YAML::Value;

  out << YAML::BeginMap;
  out << YAML::Key << "v_max" << YAML::Value << w.v_max
      << YAML::Comment("[m/s]");
  out << YAML::Key << "a_max" << YAML::Value << w.a_max
      << YAML::Comment("[m/s^2]");
  out << YAML::Key << "j_max" << YAML::Value << w.j_max
      << YAML::Comment("[m/s^3]");
  out << YAML::Key << "s_max" << YAML::Value << w.s_max
      << YAML::Comment("[m/s^4]");
  out << YAML::Key << "yaw_dot_max" << YAML::Value << w.yaw_dot_max
      << YAML::Comment("[deg/s]");

  out << YAML::Key << "vertices" << YAML::Value;
  if (!w.waypoints.empty()) {
    out << YAML::BeginSeq;
    for (typename Waypoints::WaypointVector::const_iterator it =
             w.waypoints.begin();
         it != w.waypoints.end(); ++it) {
      out << *it;
    }
    out << YAML::EndSeq;
  }

  out << YAML::EndMap;
  out << YAML::EndMap;

  return out;
}

/// Writes a mav_planning_utils::path_planning::Vertex structure to yaml.
template <int D>
YAML::Emitter& operator<<(
    YAML::Emitter& out, const mav_planning_utils::path_planning::Vertex<D>& v) {
  typedef mav_planning_utils::path_planning::Vertex<D> Vertex;
  out << YAML::BeginMap;
  out << YAML::Key << "vertex" << YAML::Value;
  out << YAML::BeginMap;
  out << YAML::Key << "time_to_next" << YAML::Value << v.time_to_next;
  out << YAML::Key << "derivative_to_optimize" << YAML::Value;
  out << mav_planning_utils::DerivativesP::toString(v.derivative_to_optimize);
  out << YAML::Key << "constraints";
  out << YAML::Comment(
      "type: derivative to be optimized. value: [m/s^type] / [deg/s^type] "
      "respectively");
  out << YAML::Value;
  out << YAML::BeginSeq;
  for (typename Vertex::Constraints::const_iterator it = v.cBegin();
       it != v.cEnd(); ++it) {
    out << YAML::BeginMap;
    out << YAML::Key << "type";
    out << YAML::Value << mav_planning_utils::DerivativesP::toString(it->first);
    out << YAML::Key << "value";
    out << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (int i = 0; i < D; ++i) {
      out << it->second[i];
    }
    out << YAML::EndSeq;
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  out << YAML::EndMap;
  return out;
}

/// Loads waypoints from file.
/**
 * @throw YAML:Exception
 */
template <int D>
void loadWaypoints(const std::string& filename, Waypoints<D>& waypoints) {
  YAML::Node doc = YAML::LoadFile(filename);

  doc >> waypoints;
}

/// Saves waypoints to file.
/**
 * @throw YAML:Exception
 */
template <int D>
void saveWaypoints(const std::string& filename, const Waypoints<D>& waypoints) {
  YAML::Emitter out;
  out << waypoints;
  std::ofstream f_out(filename.c_str());
  f_out << out.c_str();
  f_out.close();
}
}
}

#endif /* YAML_IO_H_ */
