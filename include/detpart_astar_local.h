#ifndef DETPART_ASTAR_LOCAL_H
#define DETPART_ASTAR_LOCAL_H

#include "ripup.h"

namespace sproute {

class Usage_Update {
public: 
  int grid;
  bool is_H;
  int delta;
  Usage_Update(int g, bool h, int d) {
    grid = g;
    is_H = h;
    delta = d;
  }
};

bool newRipupCheck_atomic_local(TreeEdge* treeedge, int ripup_threshold, int netID,
                          int edgeID, int* h_local_usage, int* v_local_usage);

} //namespace sproute

#endif
