#ifndef GLOBAL_H
#define GLOBAL_H

#include "flute.h"

namespace sproute {

int numgrp[10] = {0, 0, 0, 0, 6, 30, 180, 1260, 10080, 90720};

struct csoln* LUT[FLUTE_D + 1][MGROUP]; // storing 4 .. D
int numsoln[FLUTE_D + 1][MGROUP];

float round_avg_dist;
int round_avg_length;
int xGrid, yGrid, numGrids, numNets, vCapacity, hCapacity;
float vCapacity_lb, hCapacity_lb, vCapacity_ub, hCapacity_ub;
int MaxDegree;
int MinWidth[MAXLAYER], MinSpacing[MAXLAYER], ViaSpacing[MAXLAYER];
int xcorner, ycorner, wTile, hTile;
int enlarge, costheight, ripup_threshold, ahTH;
int numValidNets,
    numInvalidNets; // # nets need to be routed (having pins in different grids)
int numLayers;
int totalNumSeg;   // total # segments
int totalOverflow; // total # overflow
int mazeThreshold; // the wirelen threshold to do maze routing
Net** nets;
Net** invalid_nets;
Edge *h_edges, *v_edges;
float d1[YRANGE][XRANGE];
float d2[YRANGE][XRANGE];

/*Bool HV[YRANGE][XRANGE];
Bool hyperV[YRANGE][XRANGE];
Bool hyperH[YRANGE][XRANGE];

int corrEdge[YRANGE][XRANGE];*/ //Michael
int SLOPE;
int vCapacity3D[MAXLAYER], hCapacity3D[MAXLAYER];

float LB;
float UB;
int THRESH_M;
double LOGIS_COF;
int ENLARGE;
int STEP;
int COSHEIGHT;
int STOP;
int VCA;
float L;
int VIA, slope, max_adj;

char benchFile[STRINGLEN];

Segment* seglist;
int* seglistIndex; // the index for the segments for each net
int* seglistCnt;   // the number of segements for each net
int* segOrder;     // the order of segments for routing
Tree* trees;       // the tree topologies
StTree* sttrees;   // the Steiner trees
DTYPE** gxs;       // the copy of xs for nets, used for second FLUTE
DTYPE** gys;       // the copy of xs for nets, used for second FLUTE
DTYPE** gs; // the copy of vertical sequence for nets, used for second FLUTE
int MD = 0, TD = 0;

Edge3D* h_edges3D;
Edge3D* v_edges3D;

OrderNetPin* treeOrderPV;
OrderTree* treeOrderCong;
int numTreeedges;
int viacost;

int layerGrid[MAXLAYER][MAXLEN];
int gridD[MAXLAYER][MAXLEN];
int viaLink[MAXLAYER][MAXLEN];

int d13D[MAXLAYER][YRANGE][XRANGE];
short d23D[MAXLAYER][YRANGE][XRANGE];

dirctionT*** directions3D;
int*** corrEdge3D;
parent3D*** pr3D;

int mazeedge_Threshold;
bool inRegion[YRANGE][XRANGE];

bool heapVisited[MAXNETDEG];
// int heapQueue[MAXNETDEG]; //Michael

int gridHV, gridH, gridV, gridHs[MAXLAYER], gridVs[MAXLAYER];

int** heap13D;
short** heap23D;

float *h_costTable, *v_costTable;
bool stopDEC, errorPRONE;
// OrderNetEdge netEO[2000]; //Michael
int xcor[MAXD * 2 - 3], ycor[MAXD * 2 - 3], dcor[MAXD * 2 - 3];

StTree* sttreesBK;

short **parentX1, **parentY1, **parentX3, **parentY3;

/*float **heap2,**heap1; //Michael

Bool *pop_heap2;*/

// Michael:
int LOCK;

int max_heap_size = MAX_HEAP_SIZE;
node_pair* heap;


float costHVH[XRANGE]; // Horizontal first Z
float costVHV[YRANGE]; // Vertical first Z
float costH[YRANGE];   // Horizontal segment cost
float costV[XRANGE];   // Vertical segment cost
float costLR[YRANGE];  // Left and right boundary cost
float costTB[XRANGE];  // Top and bottom boundary cost

float costHVHtest[YRANGE]; // Vertical first Z
float costVtest[XRANGE];   // Vertical segment cost
float costTBtest[XRANGE];  // Top and bottom boundary cost

}


#endif
