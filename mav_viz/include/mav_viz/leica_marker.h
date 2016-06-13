/*
 * leica_marker.h
 *
 *  Created on: Jul 22, 2011
 *      Author: acmarkus
 */

#ifndef LEICA_MARKER_H_
#define LEICA_MARKER_H_

#include <mav_viz/marker_group.h>

namespace mav_viz {

class LeicaMarker : public MarkerGroup {
 private:
  void createLeica();
 public:
  LeicaMarker();
};

}  // end namespace mav_viz

#endif /* LEICA_MARKER_H_ */
