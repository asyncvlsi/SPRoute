#ifndef ASTAR_H
#define ASTAR_H
#include <cstddef>
#include "maze.h"

namespace sproute {

class astar_pq_grid {
public:
  float* d1_p;
  float d1_push;
  float target_dist_push;
  astar_pq_grid() {
    d1_p    = NULL;
    d1_push = 0;
    target_dist_push = 0;
  };
  astar_pq_grid(float* d1_p, float d1_push, float target_dist_push) {
    this->d1_p    = d1_p;
    this->d1_push = d1_push;
    this->target_dist_push = target_dist_push;
  }
};

typedef struct {
  bool operator()(const astar_pq_grid& left, const astar_pq_grid& right) const {
    return left.d1_push + left.target_dist_push < right.d1_push + right.target_dist_push;
  }
} astar_pq_less;

typedef galois::PerThreadMinHeap<astar_pq_grid, astar_pq_less> astar_PerThread_PQ;
typedef galois::gstl::PQ<astar_pq_grid, astar_pq_less> astar_local_pq;

// ripup a tree edge according to its ripup type and Z-route it
// put all the nodes in the subtree t1 and t2 into heap1 and heap2
// netID   - the ID for the net
// edgeID  - the ID for the tree edge to route
// d1      - the distance of any grid from the source subtree t1
// pq1     - the priority queue stored the node, cost from src, distance to dst
// v2      - the vector stored the destination nodes
void setupHeap_astar(int netID, int edgeID, astar_local_pq& pq1, local_vec& v2,
               int regionX1, int regionX2, int regionY1, int regionY2,
               float** d1, int** corrEdge, bool** inRegion, float astar_weight);



} //namespace sproute

#endif