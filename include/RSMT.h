#ifndef _RSMT_H
#define _RSMT_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "EdgeShift.h"
#include "utility.h"

namespace sproute {

#define FLUTEACCURACY 2

struct pnt {
  DTYPE x, y;
  int o;
};

// global variable
// ** V_table;
// int ** H_table;

/*static int ordery1(const void *a,  const void *b)
{
    struct wire *pa, *pb;

    pa = *(struct wire**)a;
    pb = *(struct wire**)b;

    if (pa->y1 < pb->y1) return -1;
    if (pa->y1 > pb->y1) return 1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);

}

static int ordery2(const void *a,  const void *b)
{
    struct wire *pa, *pb;

    pa = *(struct wire**)a;
    pb = *(struct wire**)b;

    if (pa->y2 < pb->y2) return -1;
    if (pa->y2 > pb->y2) return 1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);

}

static int orderx1(const void *a,  const void *b)
{
    struct wire *pa, *pb;

    pa = *(struct wire**)a;
    pb = *(struct wire**)b;

    if (pa->x1 < pb->x1) return -1;
    if (pa->x1 > pb->x1) return 1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);

}

static int orderx2(const void *a,  const void *b)
{
    struct wire *pa, *pb;

    pa = *(struct wire**)a;
    pb = *(struct wire**)b;

    if (pa->x2 < pb->x2) return -1;
    if (pa->x2 > pb->x2) return 1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);

}*/

/*int orderx(const void *a, const void *b)
{
    struct pnt *pa, *pb;

    pa = *(struct pnt**)a;
    pb = *(struct pnt**)b;

    if (pa->x < pb->x) return -1;
    if (pa->x > pb->x) return 1;
    return 0;
    //return Y(*(struct Segment*)a.x1-*(struct Segment*)b.x1);
}


static int ordery(const void *a, const void *b)
{
    struct pnt *pa, *pb;

    pa = *(struct pnt**)a;
    pb = *(struct pnt**)b;

    if (pa->y < pb->y) return -1;
    if (pa->y > pb->y) return 1;
    return 0;
}*/

// binary search to map the new coordinates to original coordinates
int mapxy(int nx, int xs[], int nxs[], int d);

void copyStTree(int ind, Tree rsmt);
void fluteNormal(int netID, int d, DTYPE x[], DTYPE y[], int acc, float coeffV,
                 Tree* t);
void fluteCongest(int netID, int d, DTYPE x[], DTYPE y[], int acc, float coeffV,
                  Tree* t);

bool netCongestion(int netID);

bool VTreeSuite(int netID);

bool HTreeSuite(int netID);

float coeffADJ(int netID);

void gen_brk_RSMT(bool congestionDriven, bool reRoute, bool genTree,
                  bool newType, bool noADJ);

} //namespace sproute

#endif
