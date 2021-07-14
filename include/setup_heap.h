#ifndef SETUP_HEAP_H
#define SETUP_HEAP_H

#include <vector>
#include "galois/LargeArray.h"
#include "a_star.h"

namespace sproute {

void setupHeap_astar_swap(int netID, int edgeID, astar_local_pq& pq1, local_vec& v2,
               int regionX1, int regionX2, int regionY1, int regionY2,
               float** d1, int** corrEdge, bool** inRegion, float astar_weight);


void setupHeap_steiner(int netID, int edgeID, astar_local_pq& pq1, local_vec& v2,
               int regionX1, int regionX2, int regionY1, int regionY2,
               float** d1, int** corrEdge, bool** inRegion, float astar_weight);

} //namespace sproute


#endif
