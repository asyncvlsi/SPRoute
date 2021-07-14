#ifndef _MAZE_H_
#define _MAZE_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <functional>
#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "route.h"
#include "ripup.h"
#include "RSMT.h"

#include "galois/Galois.h"
#include "galois/gstl.h"
#include "galois/PerThreadContainer.h"
#include "galois/Reduction.h"
#include "galois/PriorityQueue.h"
#include "galois/Timer.h"
#include "galois/graphs/Graph.h"
#include "galois/graphs/TypeTraits.h"
#include "galois/substrate/SimpleLock.h"
#include "galois/substrate/NumaMem.h"
#include "galois/AtomicHelpers.h"
#include "galois/runtime/Profile.h"

#include "galois/LargeArray.h"

// using namespace std;

namespace sproute {

#define PARENT(i) (i - 1) / 2
//#define PARENT(i) ((i-1)>>1)
#define LEFT(i) 2 * i + 1
#define RIGHT(i) 2 * i + 2

#define NET_PARALLEL 0

struct maze_greater {
  bool operator()(float* left, float* right) { return (*left) > (*right); }
};

class pq_grid {
public:
  float* d1_p;
  float d1_push;
  pq_grid() {
    d1_p    = NULL;
    d1_push = 0;
  };
  pq_grid(float* d1_p, float d1_push) {
    this->d1_p    = d1_p;
    this->d1_push = d1_push;
  }
};

class lateUpdateReq {
public:
  std::atomic<float>* d1_p;
  float d1_push;
  short parentX;
  short parentY;
  bool HV;
  lateUpdateReq() {
    d1_p    = NULL;
    d1_push = 0;
    parentX = 0;
    parentY = 0;
    HV      = false; // 1 == true, 3 == false
  };
  lateUpdateReq(std::atomic<float>* d1_p, float d1_push, short parentX,
                short parentY, bool HV) {
    this->d1_p    = d1_p;
    this->d1_push = d1_push;
    this->parentX = parentX;
    this->parentY = parentY;
    this->HV      = HV;
  }
};

struct pq_less {
  bool operator()(const pq_grid& left, const pq_grid& right) const {
    return left.d1_push < right.d1_push;
  }
};

/*typedef galois::PerThreadDeque< float* > PerThread_PQ;
typedef galois::gstl::Deque< float* > local_pq;*/ //FIFO TRIAL

typedef galois::PerThreadMinHeap<pq_grid, pq_less> PerThread_PQ;
typedef galois::gstl::PQ<pq_grid, pq_less> local_pq;

typedef galois::PerThreadVector<int> PerThread_Vec;
typedef galois::gstl::Vector<int> local_vec;

typedef struct {
  int x; // x position
  int y; // y position
} Pos;

#define FIFO_CHUNK_SIZE 4
#define OBIM_delta 20

namespace {

auto RequestIndexer = [](const pq_grid& top) {
  return (unsigned int)(top.d1_push) /
         max(OBIM_delta, (int)(costheight / (2 * slope)));
};

auto RequestIndexerLate = [](const lateUpdateReq& top) {
  return (unsigned int)(top.d1_push) / OBIM_delta;
};

}

bool checkIfDone(TreeEdge* treeedge);
/*auto RequestIndexerConcurrent = [&](const concurrent_pq_grid& top) {
    return (unsigned int)(top.d1_push) / OBIM_delta;
};*/

namespace gwl = galois::worklists;
using PSChunk = gwl::PerThreadChunkFIFO<FIFO_CHUNK_SIZE>;
using OBIM    = gwl::OrderedByIntegerMetric<decltype(RequestIndexer), PSChunk>;
using OBIM_late =
    gwl::OrderedByIntegerMetric<decltype(RequestIndexerLate), PSChunk>;
// using OBIM_concurrent =
// gwl::OrderedByIntegerMetric<decltype(RequestIndexerConcurrent), PSChunk>;

struct THREAD_LOCAL_STORAGE {
  using LAptr = galois::substrate::LAptr;
  LAptr pop_heap2_LA;
  bool* pop_heap2;

  LAptr d1_p_LA, d1_alloc_LA;
  float** d1_p;
  float* d1_alloc;

  LAptr HV_p_LA, HV_alloc_LA, hyperV_p_LA, hyperV_alloc_LA, hyperH_p_LA,
      hyperH_alloc_LA;
  bool **HV_p, **hyperV_p, **hyperH_p;
  bool *HV_alloc, *hyperV_alloc, *hyperH_alloc;

  LAptr parentX1_p_LA, parentX1_alloc_LA, parentY1_p_LA, parentY1_alloc_LA,
      parentX3_p_LA, parentX3_alloc_LA, parentY3_p_LA, parentY3_alloc_LA;
  short **parentX1_p, **parentY1_p, **parentX3_p, **parentY3_p;
  short *parentX1_alloc, *parentY1_alloc, *parentX3_alloc, *parentY3_alloc;

  LAptr corrEdge_p_LA, corrEdge_alloc_LA;
  int** corrEdge_p;
  int* corrEdge_alloc;

  LAptr inRegion_p_LA, inRegion_alloc_LA;
  bool** inRegion_p;
  bool* inRegion_alloc;

  LAptr netEO_p_LA;
  OrderNetEdge* netEO_p;

  int* v_local_usage;
  int* h_local_usage;

  // maze_pq pq1;
  // std::vector<float*> v2;
  THREAD_LOCAL_STORAGE() {
    using namespace galois::substrate;

    if (NET_PARALLEL) {
      pop_heap2_LA = largeMallocLocal(yGrid * xGrid * sizeof(bool));
      pop_heap2    = reinterpret_cast<bool*>(pop_heap2_LA.get());

      d1_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(float));
      d1_alloc    = reinterpret_cast<float*>(d1_alloc_LA.get());
      d1_p_LA     = largeMallocLocal(yGrid * sizeof(float*));
      d1_p        = reinterpret_cast<float**>(d1_p_LA.get());

      HV_alloc_LA     = largeMallocLocal(yGrid * xGrid * sizeof(bool));
      HV_alloc        = reinterpret_cast<bool*>(HV_alloc_LA.get());
      hyperV_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(bool));
      hyperV_alloc    = reinterpret_cast<bool*>(hyperV_alloc_LA.get());
      hyperH_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(bool));
      hyperH_alloc    = reinterpret_cast<bool*>(hyperH_alloc_LA.get());

      HV_p_LA     = largeMallocLocal(yGrid * sizeof(bool*));
      HV_p        = reinterpret_cast<bool**>(HV_p_LA.get());
      hyperV_p_LA = largeMallocLocal(yGrid * sizeof(bool*));
      hyperV_p    = reinterpret_cast<bool**>(hyperV_p_LA.get());
      hyperH_p_LA = largeMallocLocal(yGrid * sizeof(bool*));
      hyperH_p    = reinterpret_cast<bool**>(hyperH_p_LA.get());

      parentX1_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(short));
      parentX1_alloc    = reinterpret_cast<short*>(parentX1_alloc_LA.get());
      parentX3_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(short));
      parentX3_alloc    = reinterpret_cast<short*>(parentX3_alloc_LA.get());
      parentY1_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(short));
      parentY1_alloc    = reinterpret_cast<short*>(parentY1_alloc_LA.get());
      parentY3_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(short));
      parentY3_alloc    = reinterpret_cast<short*>(parentY1_alloc_LA.get());

      parentX1_p_LA = largeMallocLocal(yGrid * sizeof(short*));
      parentX1_p    = reinterpret_cast<short**>(parentX1_p_LA.get());
      parentX3_p_LA = largeMallocLocal(yGrid * sizeof(short*));
      parentX3_p    = reinterpret_cast<short**>(parentX3_p_LA.get());
      parentY1_p_LA = largeMallocLocal(yGrid * sizeof(short*));
      parentY1_p    = reinterpret_cast<short**>(parentY1_p_LA.get());
      parentY3_p_LA = largeMallocLocal(yGrid * sizeof(short*));
      parentY3_p    = reinterpret_cast<short**>(parentY3_p_LA.get());

      corrEdge_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(int));
      corrEdge_alloc    = reinterpret_cast<int*>(corrEdge_alloc_LA.get());
      corrEdge_p_LA     = largeMallocLocal(yGrid * sizeof(int*));
      corrEdge_p        = reinterpret_cast<int**>(corrEdge_p_LA.get());

      inRegion_alloc_LA = largeMallocLocal(yGrid * xGrid * sizeof(bool));
      inRegion_alloc    = reinterpret_cast<bool*>(inRegion_alloc_LA.get());
      inRegion_p_LA     = largeMallocLocal(yGrid * sizeof(bool*));
      inRegion_p        = reinterpret_cast<bool**>(inRegion_p_LA.get());

      netEO_p_LA = largeMallocLocal(MAXNETDEG * 2 * sizeof(OrderNetEdge));
      netEO_p    = reinterpret_cast<OrderNetEdge*>(netEO_p_LA.get());


    } else {
      pop_heap2 = (bool*)calloc(yGrid * xGrid, sizeof(bool));

      d1_alloc = (float*)calloc(yGrid * xGrid, sizeof(float));
      d1_p     = (float**)calloc(yGrid, sizeof(float*));

      HV_alloc     = (bool*)calloc(yGrid * xGrid, sizeof(bool));
      hyperV_alloc = (bool*)calloc(yGrid * xGrid, sizeof(bool));
      hyperH_alloc = (bool*)calloc(yGrid * xGrid, sizeof(bool));
      HV_p         = (bool**)calloc(yGrid, sizeof(bool*));
      hyperV_p     = (bool**)calloc(yGrid, sizeof(bool*));
      hyperH_p     = (bool**)calloc(yGrid, sizeof(bool*));

      parentX1_alloc = (short*)calloc(yGrid * xGrid, sizeof(short));
      parentX3_alloc = (short*)calloc(yGrid * xGrid, sizeof(short));
      parentY1_alloc = (short*)calloc(yGrid * xGrid, sizeof(short));
      parentY3_alloc = (short*)calloc(yGrid * xGrid, sizeof(short));
      parentX1_p     = (short**)calloc(yGrid, sizeof(short*));
      parentX3_p     = (short**)calloc(yGrid, sizeof(short*));
      parentY1_p     = (short**)calloc(yGrid, sizeof(short*));
      parentY3_p     = (short**)calloc(yGrid, sizeof(short*));

      corrEdge_alloc = (int*)calloc(yGrid * xGrid, sizeof(int));
      corrEdge_p     = (int**)calloc(yGrid, sizeof(int*));

      inRegion_alloc = (bool*)calloc(yGrid * xGrid, sizeof(bool));
      inRegion_p     = (bool**)calloc(yGrid, sizeof(bool*));

      netEO_p = (OrderNetEdge*)calloc(MAXNETDEG * 2, sizeof(OrderNetEdge));

      v_local_usage = (int*)calloc(xGrid * (yGrid - 1), sizeof(int));
      h_local_usage =  (int*)calloc((xGrid - 1) * yGrid, sizeof(int));
    }
    // printf("allocation success\n");
    for (int i = 0; i < yGrid; i++) {
      d1_p[i] = &(d1_alloc[i * xGrid]);

      HV_p[i]     = &(HV_alloc[i * xGrid]);
      hyperV_p[i] = &(hyperV_alloc[i * xGrid]);
      hyperH_p[i] = &(hyperH_alloc[i * xGrid]);

      corrEdge_p[i] = &(corrEdge_alloc[i * xGrid]);

      inRegion_p[i] = &(inRegion_alloc[i * xGrid]);
    }

    for (int i = 0; i < yGrid; i++) {
      parentX1_p[i] = &(parentX1_alloc[i * xGrid]);
      parentX3_p[i] = &(parentX3_alloc[i * xGrid]);
      parentY1_p[i] = &(parentY1_alloc[i * xGrid]);
      parentY3_p[i] = &(parentY3_alloc[i * xGrid]);
    }
  }
  void reset_heap() { memset(pop_heap2, 0, yGrid * xGrid * sizeof(bool)); }

  ~THREAD_LOCAL_STORAGE() {
    free(pop_heap2);

    free(d1_p);
    free(d1_alloc);

    free(HV_p);
    free(hyperV_p);
    free(hyperH_p);
    free(HV_alloc);
    free(hyperV_alloc);
    free(hyperH_alloc);

    free(parentX1_p);
    free(parentY1_p);
    free(parentX3_p);
    free(parentY3_p);

    free(parentX1_alloc);
    free(parentY1_alloc);
    free(parentX3_alloc);
    free(parentY3_alloc);

    free(corrEdge_alloc);
    free(corrEdge_p);

    free(inRegion_alloc);
    free(inRegion_p);

    free(netEO_p);

    free(h_local_usage);
    free(v_local_usage);
  }
};

void convertToMazerouteNet(int netID);

void convertToMazeroute();

// non recursive version of heapify
void heapify(float** array, int heapSize, int i);
// build heap for an list of grid
/*void buildHeap(float **array, int arrayLen)
{
    int i;

    for (i=arrayLen/2-1; i>=0; i--)
        heapify(array, arrayLen, i);
}*/

void updateHeap(float** array, int i);

// extract the entry with minimum distance from Priority queue
void extractMin(float** array, int arrayLen);
/*
 * num_iteration : the total number of iterations for maze route to run
 * round : the number of maze route stages runned
 */

void updateCongestionHistory(int upType);

// ripup a tree edge according to its ripup type and Z-route it
// put all the nodes in the subtree t1 and t2 into heap1 and heap2
// netID   - the ID for the net
// edgeID  - the ID for the tree edge to route
// d1      - the distance of any grid from the source subtree t1
// d2      - the distance of any grid from the destination subtree t2
// heap1   - the heap storing the addresses for d1[][]
// heap2   - the heap storing the addresses for d2[][]
void setupHeap(int netID, int edgeID, local_pq& pq1, local_vec& v2,
               int regionX1, int regionX2, int regionY1, int regionY2,
               float** d1, int** corrEdge, bool** inRegion);

int copyGrids(TreeNode* treenodes, int n1, TreeEdge* treeedges, int edge_n1n2,
              int* gridsX_n1n2, int* gridsY_n1n2);

void updateRouteType1(TreeNode* treenodes, int n1, int A1, int A2, int E1x,
                      int E1y, TreeEdge* treeedges, int edge_n1A1,
                      int edge_n1A2);

void updateRouteType2(TreeNode* treenodes, int n1, int A1, int A2, int C1,
                      int C2, int E1x, int E1y, TreeEdge* treeedges,
                      int edge_n1A1, int edge_n1A2, int edge_C1C2);

void reInitTree(int netID);

int getOverflow2Dmaze(int* maxOverflow, int* tUsage, bool after_maze = true);
void checkUsageCorrectness();

int getOverflow2D(int* maxOverflow);
int getOverflow3D(void);

void InitEstUsage();

void str_accu(int rnd);

void InitLastUsage(int upType);

} //namespace sproute

#endif
