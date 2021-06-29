#ifndef _ROUTE_H_
#define _ROUTE_H_

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "ripup.h"
#include "global_variable_extern.h"

namespace sproute {

#define SAMEX 0
#define SAMEY 1

extern float costHVH[XRANGE]; // Horizontal first Z
extern float costVHV[YRANGE]; // Vertical first Z
extern float costH[YRANGE];   // Horizontal segment cost
extern float costV[XRANGE];   // Vertical segment cost
extern float costLR[YRANGE];  // Left and right boundary cost
extern float costTB[XRANGE];  // Top and bottom boundary cost

extern float costHVHtest[YRANGE]; // Vertical first Z
extern float costVtest[XRANGE];   // Vertical segment cost
extern float costTBtest[XRANGE];  // Top and bottom boundary cost

#define HCOST 5000;

// estimate the routing by assigning 1 for H and V segments, 0.5 to both
// possible L for L segments
void estimateOneSeg(Segment* seg);

void routeSegV(Segment* seg);
// L-route, based on previous L route
void routeSegL(Segment* seg);
// First time L-route, based on 0.5-0.5 estimation
void routeSegLFirstTime(Segment* seg);
// route all segments with L, firstTime: true, no previous route, false -
// previous is L-route
void routeLAll(bool firstTime);
// L-route, rip-up the previous route according to the ripuptype
// L-route, rip-up the previous route according to the ripuptype
void newrouteL(int netID, RouteType ripuptype, bool viaGuided);

// route all segments with L, firstTime: true, first newrouteLAll, false - not
// first
void newrouteLAll(bool firstTime, bool viaGuided);

void newrouteZ_edge(int netID, int edgeID);
// Z-route, rip-up the previous route according to the ripuptype
void newrouteZ(int netID, int threshold);
// ripup a tree edge according to its ripup type and Z-route it
// route all segments with L, firstTime: true, first newrouteLAll, false - not
// first
void newrouteZAll(int threshold);

// Ripup the original route and do Monotonic routing within bounding box
void routeMonotonic(int netID, int edgeID, int threshold);
void routeMonotonicAll(int threshold);

void spiralRoute(int netID, int edgeID);
void spiralRouteAll();
void routeLVEnew(int netID, int edgeID, int threshold, int enlarge);
void routeLVAll(int threshold, int expand);
void newrouteLInMaze(int netID);

} //namespace sproute

#endif
