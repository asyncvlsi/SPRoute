#ifndef GLOBAL_VARIABLE_EXTERN_H
#define GLOBAL_VARIABLE_EXTERN_H

#include "DataType.h"
#include "DataProc.h"
#include "flute.h"

namespace sproute {

extern int numgrp[10];

extern struct csoln* LUT[FLUTE_D + 1][MGROUP]; // storing 4 .. D
extern int numsoln[FLUTE_D + 1][MGROUP];

extern float round_avg_dist;
extern int round_avg_length;
extern int xGrid, yGrid, numGrids, numNets, vCapacity, hCapacity;
extern float vCapacity_lb, hCapacity_lb, vCapacity_ub, hCapacity_ub;
extern int MaxDegree;
extern int MinWidth[MAXLAYER], MinSpacing[MAXLAYER], ViaSpacing[MAXLAYER];
extern int xcorner, ycorner, wTile, hTile;
extern int enlarge, costheight, ripup_threshold, ahTH;
extern int numValidNets,
    numInvalidNets; // # nets need to be routed (having pins in different grids)
extern int numLayers;
extern int totalNumSeg;   // total # segments
extern int totalOverflow; // total # overflow
extern int mazeThreshold; // the wirelen threshold to do maze routing

extern Net** nets;
extern Net** invalid_nets;
extern Edge *h_edges, *v_edges;
extern float d1[YRANGE][XRANGE];
extern float d2[YRANGE][XRANGE];

extern int SLOPE;
extern int vCapacity3D[MAXLAYER], hCapacity3D[MAXLAYER];

extern float LB;
extern float UB;
extern int THRESH_M;
extern double LOGIS_COF;
extern int ENLARGE;
extern int STEP;
extern int COSHEIGHT;
extern int STOP;
extern int VCA;
extern float L;
extern int VIA, slope, max_adj;

extern char benchFile[STRINGLEN];

extern Segment* seglist;
extern int* seglistIndex; // the index for the segments for each net
extern int* seglistCnt;   // the number of segements for each net
extern int* segOrder;     // the order of segments for routing
extern Tree* trees;       // the tree topologies
extern StTree* sttrees;   // the Steiner trees
extern DTYPE** gxs;       // the copy of xs for nets, used for second FLUTE
extern DTYPE** gys;       // the copy of xs for nets, used for second FLUTE
extern DTYPE** gs; // the copy of vertical sequence for nets, used for second FLUTE
extern int MD, TD;

extern Edge3D* h_edges3D;
extern Edge3D* v_edges3D;

extern OrderNetPin* treeOrderPV;
extern OrderTree* treeOrderCong;
extern int numTreeedges;
extern int viacost;

extern int layerGrid[MAXLAYER][MAXLEN];
extern int gridD[MAXLAYER][MAXLEN];
extern int viaLink[MAXLAYER][MAXLEN];

extern int d13D[MAXLAYER][YRANGE][XRANGE];
extern short d23D[MAXLAYER][YRANGE][XRANGE];

extern dirctionT*** directions3D;
extern int*** corrEdge3D;
extern parent3D*** pr3D;

extern int mazeedge_Threshold;
extern bool inRegion[YRANGE][XRANGE];

extern bool heapVisited[MAXNETDEG];
// int heapQueue[MAXNETDEG]; //Michael

extern int gridHV, gridH, gridV, gridHs[MAXLAYER], gridVs[MAXLAYER];

extern int** heap13D;
extern short** heap23D;

extern float *h_costTable, *v_costTable;
extern bool stopDEC, errorPRONE;
// OrderNetEdge netEO[2000]; //Michael
extern int xcor[MAXD * 2 - 3], ycor[MAXD * 2 - 3], dcor[MAXD * 2 - 3];

extern StTree* sttreesBK;

extern short **parentX1, **parentY1, **parentX3, **parentY3;

/*float **heap2,**heap1; //Michael

Bool *pop_heap2;*/

// Michael:
extern int LOCK;

extern int max_heap_size;

}


#endif
