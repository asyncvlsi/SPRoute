#ifndef _FLUTE_H_
#define _FLUTE_H_

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <string>
#include <iostream>

namespace sproute {
//#include "flute_mst.h"

/*****************************/
/*  User-Defined Parameters  */
/*****************************/
#define MAXD 5000              // max. degree that can be handled
#define ACCURACY 10            // Default accuracy
#define LOCAL_REFINEMENT 1     // Suggestion: Set to 1 if ACCURACY >= 5
#define REMOVE_DUPLICATE_PIN 1 // Remove dup. pin for flute_wl() & flute()

#ifndef DTYPE // Data type for distance
#define DTYPE int
#endif

/*****************************/
/*  User-Callable Functions  */
/*****************************/
// void readLUT();
// DTYPE flute_wl(int d, DTYPE x[], DTYPE y[], int acc);
// DTYPE flutes_wl(int d, DTYPE xs[], DTYPE ys[], int s[], int acc);
// Tree flute(int d, DTYPE x[], DTYPE y[], int acc);
// Tree flutes(int d, DTYPE xs[], DTYPE ys[], int s[], int acc);
// DTYPE wirelength(Tree t);
// void printtree(Tree t);
// void plottree(Tree t);

/*************************************/
/* Internal Parameters and Functions */
/*************************************/
#define POWVFILE "POWV9.dat" // LUT for POWV (Wirelength Vector)
#define POSTFILE "POST9.dat" // LUT for POST (Steiner Tree)
#define FLUTE_D 9                   // LUT is used for d <= FLUTE_D , FLUTE_D <= 9
#define TAU(A) (8 + 1.3 * (A))
#define D1(A) (25 + 120 / ((A) * (A))) // flute_mr is used for D1 < d <= D2
#define D2(A) ((A) <= 6 ? 500 : 75 + 5 * (A))

typedef struct {
  DTYPE x, y; // starting point of the branch
  int n;      // index of neighbor
} Branch;

typedef struct {
  int deg;        // degree
  DTYPE length;   // total wirelength
  Branch* branch; // array of tree branches
} Tree;

#if REMOVE_DUPLICATE_PIN == 1
#define flutes_wl(d, xs, ys, s, acc) flutes_wl_RDP(d, xs, ys, s, acc)
#define flutes(d, xs, ys, s, acc) flutes_RDP(d, xs, ys, s, acc)
#else
#define flutes_wl(d, xs, ys, s, acc) flutes_wl_ALLD(d, xs, ys, s, acc)
#define flutes(d, xs, ys, s, acc) flutes_ALLD(d, xs, ys, s, acc)
#endif

#define flutes_wl_ALLD(d, xs, ys, s, acc) flutes_wl_LMD(d, xs, ys, s, acc)
#define flutes_ALLD(d, xs, ys, s, acc)                                         \
  (d <= FLUTE_D? flutes_LD(d, xs, ys, s) : flutes_MD(d, xs, ys, s, acc))
//          : (d<=D1(acc) ? flutes_MD(d, xs, ys, s, acc)
//                        : flutes_HD(d, xs, ys, s, acc)))

#define flutes_wl_LMD(d, xs, ys, s, acc)                                       \
  (d <=FLUTE_D? flutes_wl_LD(d, xs, ys, s) : flutes_wl_MD(d, xs, ys, s, acc))
#define flutes_LMD(d, xs, ys, s, acc)                                          \
  (d <=FLUTE_D? flutes_LD(d, xs, ys, s) : flutes_MD(d, xs, ys, s, acc))

//#define max(x,y) ((x)>(y)?(x):(y))
//#define min(x,y) ((x)<(y)?(x):(y))
// to work around max conflict with bitmap
//#define abs(x) ((x)<0?(-x):(x))
using namespace std;
#define ADIFF(x, y) ((x) > (y) ? (x - y) : (y - x)) // Absolute difference

#if FLUTE_D<= 7
#define MGROUP 5040 / 4 // Max. # of groups, 7! = 5040
#define MPOWV 15        // Max. # of POWVs per group
#elif FLUTE_D== 8
#define MGROUP 40320 / 4 // Max. # of groups, 8! = 40320
#define MPOWV 33         // Max. # of POWVs per group
#elif FLUTE_D== 9
#define MGROUP 362880 / 4 // Max. # of groups, 9! = 362880
#define MPOWV 79          // Max. # of POWVs per group
#endif
extern int numgrp[10];

struct csoln {
  unsigned char parent;
  unsigned char seg[11];       // Add: 0..i, Sub: j..10; seg[i+1]=seg[j-1]=0
  unsigned char rowcol[FLUTE_D - 2]; // row = rowcol[]/16, col = rowcol[]%16,
  unsigned char neighbor[2 * FLUTE_D - 2];
};
extern struct csoln* LUT[FLUTE_D + 1][MGROUP]; // storing 4 .. D
extern int numsoln[FLUTE_D + 1][MGROUP];

typedef struct node_pair_s { // pair of nodes representing an edge
  int node1, node2;
} node_pair;
extern node_pair* heap;

struct point_flute {
  DTYPE x, y;
  int o;
};

void readLUT();
DTYPE flute_wl(int d, DTYPE x[], DTYPE y[], int acc);
DTYPE flutes_wl_LD(int d, DTYPE xs[], DTYPE ys[], int s[]);
DTYPE flutes_wl_MD(int d, DTYPE xs[], DTYPE ys[], int s[], int acc);
DTYPE flutes_wl_RDP(int d, DTYPE xs[], DTYPE ys[], int s[], int acc);
Tree flute(int d, DTYPE x[], DTYPE y[], int acc);
Tree flutes_LD(int d, DTYPE xs[], DTYPE ys[], int s[]);
Tree flutes_MD(int d, DTYPE xs[], DTYPE ys[], int s[], int acc);
Tree flutes_RDP(int d, DTYPE xs[], DTYPE ys[], int s[], int acc);
Tree dmergetree(Tree t1, Tree t2);
Tree hmergetree(Tree t1, Tree t2, int s[]);
Tree vmergetree(Tree t1, Tree t2);
void local_refinement(Tree* tp, int p);
DTYPE wirelength(Tree t);
void printtree(Tree t);
void plottree(Tree t);

#define MAX_HEAP_SIZE (MAXD * 2)
extern int max_heap_size;
inline void init_param() {
  heap = (node_pair*)malloc(sizeof(node_pair) * (max_heap_size + 1));
}

[[noreturn]] inline void abort_with_message(std::string message) noexcept {
  std::cerr << message << std::endl;
  std::abort();
}

void readLUT(const char* fluteDir);

static int orderx(const void* a, const void* b) {
  struct point_flute *pa, *pb;

  pa = *(struct point_flute**)a;
  pb = *(struct point_flute**)b;

  if (pa->x < pb->x)
    return -1;
  if (pa->x > pb->x)
    return 1;
  return 0;
}

static int ordery(const void* a, const void* b) {
  struct point_flute *pa, *pb;

  pa = *(struct point_flute**)a;
  pb = *(struct point_flute**)b;

  if (pa->y < pb->y)
    return -1;
  if (pa->y > pb->y)
    return 1;
  return 0;
}

} //namespace sproute


#endif /* _FLUTE_H_ */


