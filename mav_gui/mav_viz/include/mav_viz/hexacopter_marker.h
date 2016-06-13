/*
 * hexacopter_marker.h
 *
 *  Created on: Jul 22, 2011
 *      Author: acmarkus
 */

#ifndef HEXACOPTER_MARKER_H_
#define HEXACOPTER_MARKER_H_

#include "marker_group.h"

namespace mav_viz {

class HexacopterMarker : public MarkerGroup {
 private:
  void createHexacopter(bool simple = false);
 public:
  HexacopterMarker(bool simple = false);
};

}  // end namespace mav_viz
#endif /* HEXACOPTER_MARKER_H_ */
