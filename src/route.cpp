#include "route.h"
#include "sproute.h"

namespace sproute {
void estimateOneSeg(Segment* seg) {
  int i;
  int ymin, ymax;

  if (seg->y1 < seg->y2) {
    ymin = seg->y1;
    ymax = seg->y2;
  } else {
    ymin = seg->y2;
    ymax = seg->y1;
  }

  // assign 0.5 to both Ls (x1,y1)-(x1,y2) + (x1,y2)-(x2,y2) + (x1,y1)-(x2,y1) +
  // (x2,y1)-(x2,y2)
  if (seg->x1 == seg->x2) // a vertical segment
  {
    for (i = ymin; i < ymax; i++)
      v_edges[i * xGrid + seg->x1].est_usage += 1;
  } else if (seg->y1 == seg->y2) // a horizontal segment
  {
    for (i = seg->x1; i < seg->x2; i++)
      h_edges[seg->y1 * (xGrid - 1) + i].est_usage += 1;
  } else // a diagonal segment
  {
    for (i = ymin; i < ymax; i++) {
      v_edges[i * xGrid + seg->x1].est_usage += 0.5;
      v_edges[i * xGrid + seg->x2].est_usage += 0.5;
    }
    for (i = seg->x1; i < seg->x2; i++) {
      h_edges[seg->y1 * (xGrid - 1) + i].est_usage += 0.5;
      h_edges[seg->y2 * (xGrid - 1) + i].est_usage += 0.5;
    }
  }
}

void routeSegV(Segment* seg) {
  int i;
  int ymin, ymax;

  if (seg->y1 < seg->y2) {
    ymin = seg->y1;
    ymax = seg->y2;
  } else {
    ymin = seg->y2;
    ymax = seg->y1;
  }

  for (i = ymin; i < ymax; i++)
    v_edges[i * xGrid + seg->x1].est_usage++;
}

void routeSegH(Segment* seg) {
  int i;

  for (i = seg->x1; i < seg->x2; i++)
    h_edges[seg->y1 * (xGrid - 1) + i].est_usage++;
}

// L-route, based on previous L route
void routeSegL(Segment* seg) {
  int i, grid, grid1;
  float costL1, costL2, tmp;
  int ymin, ymax;

  if (seg->y1 < seg->y2) {
    ymin = seg->y1;
    ymax = seg->y2;
  } else {
    ymin = seg->y2;
    ymax = seg->y1;
  }

  if (seg->x1 == seg->x2) // V route
    routeSegV(seg);
  else if (seg->y1 == seg->y2) // H route
    routeSegH(seg);
  else // L route
  {
    costL1 = costL2 = 0;

    for (i = ymin; i < ymax; i++) {
      grid = i * xGrid;
      tmp  = v_edges[grid + seg->x1].red + v_edges[grid + seg->x1].est_usage -
            vCapacity_lb;
      if (tmp > 0)
        costL1 += tmp;
      tmp = v_edges[grid + seg->x2].red + v_edges[grid + seg->x2].est_usage -
            vCapacity_lb;
      if (tmp > 0)
        costL2 += tmp;
    }
    grid  = seg->y2 * (xGrid - 1);
    grid1 = seg->y1 * (xGrid - 1);
    for (i = seg->x1; i < seg->x2; i++) {
      tmp = h_edges[grid + i].red + h_edges[grid + i].est_usage - hCapacity_lb;
      if (tmp > 0)
        costL1 += tmp;
      tmp =
          h_edges[grid1 + i].red + h_edges[grid1 + i].est_usage - hCapacity_lb;
      if (tmp > 0)
        costL2 += tmp;
    }

    printf("costL1 is %f, costL2 is %f\n", costL1, costL2);

    if (costL1 < costL2) {
      // two parts (x1, y1)-(x1, y2) and (x1, y2)-(x2, y2)
      for (i = ymin; i < ymax; i++) {
        v_edges[i * xGrid + seg->x1].est_usage += 1;
      }
      grid = seg->y2 * (xGrid - 1);
      for (i = seg->x1; i < seg->x2; i++) {
        h_edges[grid + i].est_usage += 1;
      }
      seg->xFirst = false;
    } // if costL1<costL2
    else {
      // two parts (x1, y1)-(x2, y1) and (x2, y1)-(x2, y2)
      grid = seg->y1 * (xGrid - 1);
      for (i = seg->x1; i < seg->x2; i++) {
        h_edges[grid + i].est_usage += 1;
      }
      for (i = ymin; i < ymax; i++) {
        v_edges[i * xGrid + seg->x2].est_usage += 1;
      }
      seg->xFirst = true;
    }
  } // else L route
}

// First time L-route, based on 0.5-0.5 estimation
void routeSegLFirstTime(Segment* seg) {
  int i, vedge, hedge;
  float costL1, costL2, tmp;
  int ymin, ymax;

  if (seg->y1 < seg->y2) {
    ymin = seg->y1;
    ymax = seg->y2;
  } else {
    ymin = seg->y2;
    ymax = seg->y1;
  }

  costL1 = costL2 = 0;

  for (i = ymin; i < ymax; i++) {
    vedge = i * xGrid + seg->x1;
    tmp   = v_edges[vedge].red + v_edges[vedge].est_usage - vCapacity_lb;
    if (tmp > 0)
      costL1 += tmp;
  }
  for (i = ymin; i < ymax; i++) {
    vedge = i * xGrid + seg->x2;
    tmp   = v_edges[vedge].red + v_edges[vedge].est_usage - vCapacity_lb;
    if (tmp > 0)
      costL2 += tmp;
  }

  for (i = seg->x1; i < seg->x2; i++) {
    hedge = seg->y2 * (xGrid - 1) + i;
    tmp   = h_edges[hedge].red + h_edges[hedge].est_usage - hCapacity_lb;
    if (tmp > 0)
      costL1 += tmp;
  }
  for (i = seg->x1; i < seg->x2; i++) {
    hedge = seg->y1 * (xGrid - 1) + i;
    tmp   = h_edges[hedge].red + h_edges[hedge].est_usage - hCapacity_lb;
    if (tmp > 0)
      costL2 += tmp;
  }

  if (costL1 < costL2) {
    // two parts (x1, y1)-(x1, y2) and (x1, y2)-(x2, y2)
    for (i = ymin; i < ymax; i++) {
      vedge = i * xGrid + seg->x1;
      v_edges[vedge].est_usage += 0.5;
      vedge += seg->x2 - seg->x1;
      v_edges[vedge].est_usage -= 0.5;
    }
    for (i = seg->x1; i < seg->x2; i++) {
      hedge = seg->y2 * (xGrid - 1) + i;
      h_edges[hedge].est_usage += 0.5;
      hedge = seg->y1 * (xGrid - 1) + i;
      h_edges[hedge].est_usage -= 0.5;
    }
    seg->xFirst = false;
  } else {
    // two parts (x1, y1)-(x2, y1) and (x2, y1)-(x2, y2)
    for (i = seg->x1; i < seg->x2; i++) {
      hedge = seg->y1 * (xGrid - 1) + i;
      h_edges[hedge].est_usage += 0.5;
      hedge = seg->y2 * (xGrid - 1) + i;
      h_edges[hedge].est_usage -= 0.5;
    }
    for (i = ymin; i < ymax; i++) {
      vedge = i * xGrid + seg->x2;
      v_edges[vedge].est_usage += 0.5;
      vedge += seg->x1 - seg->x2;
      v_edges[vedge].est_usage -= 0.5;
    }
    seg->xFirst = true;
  }
}

// route all segments with L, firstTime: true, no previous route, false -
// previous is L-route
void routeLAll(bool firstTime) {
  int i, j;

  if (firstTime) // no previous route
  {
    // estimate congestion with 0.5+0.5 L
    for (i = 0; i < numValidNets; i++) {
      for (j = seglistIndex[i]; j < seglistIndex[i] + seglistCnt[i]; j++) {
        estimateOneSeg(&seglist[j]);
      }
    }
    // L route
    for (i = 0; i < numValidNets; i++) {
      for (j = seglistIndex[i]; j < seglistIndex[i] + seglistCnt[i]; j++) {
        // no need to reroute the H or V segs
        if (seglist[j].x1 != seglist[j].x2 || seglist[j].y1 != seglist[j].y2)
          routeSegLFirstTime(&seglist[j]);
      }
    }
  } else // previous is L-route
  {
    for (i = 0; i < numValidNets; i++) {
      for (j = seglistIndex[i]; j < seglistIndex[i] + seglistCnt[i]; j++) {
        // no need to reroute the H or V segs
        if (seglist[j].x1 != seglist[j].x2 || seglist[j].y1 != seglist[j].y2) {
          ripupSegL(&seglist[j]);
          routeSegL(&seglist[j]);
        }
      }
    }
  }
}

// L-route, rip-up the previous route according to the ripuptype
// L-route, rip-up the previous route according to the ripuptype
void newrouteL(int netID, RouteType ripuptype, bool viaGuided) {
  int i, j, d, n1, n2, x1, y1, x2, y2, grid, grid1;
  float costL1 = 0, costL2 = 0, tmp;
  int ymin, ymax;
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  d         = sttrees[netID].deg;
  treeedges = sttrees[netID].edges;
  treenodes = sttrees[netID].nodes;

  // loop for all the tree edges (2*d-3)
  for (i = 0; i < 2 * d - 3; i++) {
    if (sttrees[netID].edges[i].len >
        0) // only route the non-degraded edges (len>0)
    {

      treeedge = &(treeedges[i]);

      n1 = treeedge->n1;
      n2 = treeedge->n2;
      x1 = treenodes[n1].x;
      y1 = treenodes[n1].y;
      x2 = treenodes[n2].x;
      y2 = treenodes[n2].y;

      if (y1 < y2) {
        ymin = y1;
        ymax = y2;
      } else {
        ymin = y2;
        ymax = y1;
      }

      // ripup the original routing
      if (ripuptype > NOROUTE) // it's been routed
        newRipup(treeedge, x1, y1, x2, y2);

      treeedge->route.type = LROUTE;
      if (x1 == x2) // V-routing
      {
        for (j = ymin; j < ymax; j++)
          v_edges[j * xGrid + x1].est_usage++;
        treeedge->route.xFirst = false;
        if (treenodes[n1].status % 2 == 0) {
          treenodes[n1].status += 1;
        }
        if (treenodes[n2].status % 2 == 0) {
          treenodes[n2].status += 1;
        }
      } else if (y1 == y2) // H-routing
      {
        for (j = x1; j < x2; j++)
          h_edges[y1 * (xGrid - 1) + j].est_usage++;
        treeedge->route.xFirst = true;
        if (treenodes[n2].status < 2) {
          treenodes[n2].status += 2;
        }
        if (treenodes[n1].status < 2) {
          treenodes[n1].status += 2;
        }
      } else // L-routing
      {

        if (viaGuided) {

          if (treenodes[n1].status == 0 || treenodes[n1].status == 3) {
            costL1 = costL2 = 0;
          } else if (treenodes[n1].status == 2) {
            costL1 = viacost;
            costL2 = 0;
          } else if (treenodes[n1].status == 1) {

            costL1 = 0;
            costL2 = viacost;
          } else {
            printf("wrong node status %d", treenodes[n1].status);
          }
          if (treenodes[n2].status == 2) {
            costL2 += viacost;
          } else if (treenodes[n2].status == 1) {
            costL1 += viacost;
          }
        } else {
          costL1 = costL2 = 0;
        }

        for (j = ymin; j < ymax; j++) {
          grid = j * xGrid;
          tmp  = v_edges[grid + x1].est_usage - vCapacity_lb +
                v_edges[grid + x1].red;
          if (tmp > 0)
            costL1 += tmp;
          tmp = v_edges[grid + x2].est_usage - vCapacity_lb +
                v_edges[grid + x2].red;
          if (tmp > 0)
            costL2 += tmp;
        }
        grid  = y2 * (xGrid - 1);
        grid1 = y1 * (xGrid - 1);
        for (j = x1; j < x2; j++) {
          tmp = h_edges[grid + j].est_usage - hCapacity_lb +
                h_edges[grid + j].red;
          if (tmp > 0)
            costL1 += tmp;
          tmp = h_edges[grid1 + j].est_usage - hCapacity_lb +
                h_edges[grid1 + j].red;
          if (tmp > 0)
            costL2 += tmp;
        }

        if (costL1 < costL2) {
          if (treenodes[n1].status % 2 == 0) {
            treenodes[n1].status += 1;
          }
          if (treenodes[n2].status < 2) {
            treenodes[n2].status += 2;
          }

          // two parts (x1, y1)-(x1, y2) and (x1, y2)-(x2, y2)
          for (j = ymin; j < ymax; j++) {
            v_edges[j * xGrid + x1].est_usage += 1;
          }
          grid = y2 * (xGrid - 1);
          for (j = x1; j < x2; j++) {
            h_edges[grid + j].est_usage += 1;
          }
          treeedge->route.xFirst = false;
        } // if costL1<costL2
        else {
          if (treenodes[n2].status % 2 == 0) {
            treenodes[n2].status += 1;
          }
          if (treenodes[n1].status < 2) {
            treenodes[n1].status += 2;
          }

          // two parts (x1, y1)-(x2, y1) and (x2, y1)-(x2, y2)
          grid = y1 * (xGrid - 1);
          for (j = x1; j < x2; j++) {
            h_edges[grid + j].est_usage += 1;
          }
          for (j = ymin; j < ymax; j++) {
            v_edges[j * xGrid + x2].est_usage += 1;
          }
          treeedge->route.xFirst = true;
        }

      } // else L-routing
    }   // if non-degraded edge
    else
      sttrees[netID].edges[i].route.type = NOROUTE;
  } // loop i
}

// route all segments with L, firstTime: true, first newrouteLAll, false - not
// first
void newrouteLAll(bool firstTime, bool viaGuided) {
  int i;

  if (firstTime) {
    for (i = 0; i < numValidNets; i++) {
      newrouteL(i, NOROUTE, viaGuided); // do L-routing
    }
  } else {
    for (i = 0; i < numValidNets; i++) {
      newrouteL(i, LROUTE, viaGuided);
    }
  }
}

void newrouteZ_edge(int netID, int edgeID) {
  int i, j, n1, n2, x1, y1, x2, y2, segWidth, bestZ, grid, grid1, grid2, ymin,
      ymax;
  float tmp, bestcost, btTEST;
  bool HVH; // the shape of Z routing (true - HVH, false - VHV)
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  if (sttrees[netID].edges[edgeID].len >
      0) // only route the non-degraded edges (len>0)
  {
    treeedges = sttrees[netID].edges;
    treeedge  = &(treeedges[edgeID]);
    treenodes = sttrees[netID].nodes;
    n1        = treeedge->n1;
    n2        = treeedge->n2;
    x1        = treenodes[n1].x;
    y1        = treenodes[n1].y;
    x2        = treenodes[n2].x;
    y2        = treenodes[n2].y;

    if (x1 != x2 || y1 != y2) // not H or V edge, do Z-routing (if H or V edge,
                              // no need to reroute)
    {
      // ripup the original routing
      newRipup(treeedge, x1, y1, x2, y2);

      treeedge->route.type = ZROUTE;

      segWidth = x2 - x1;
      if (y1 < y2) {
        ymin = y1;
        ymax = y2;
      } else {
        ymin = y2;
        ymax = y1;
      }

      // compute the cost for all Z routing

      for (i = 0; i <= segWidth; i++) {
        costHVH[i] = 0;
        costV[i]   = 0;
        costTB[i]  = 0;

        costHVHtest[i] = 0;
        costVtest[i]   = 0;
        costTBtest[i]  = 0;
      }

      // compute the cost for all H-segs and V-segs and partial boundary seg
      // cost for V-segs
      for (i = x1; i <= x2; i++) {
        grid = ymin * xGrid;
        for (j = ymin; j < ymax; j++) {
          tmp = v_edges[grid + i].est_usage - vCapacity_lb +
                v_edges[grid + i].red;
          grid += xGrid;
          if (tmp > 0) {
            costV[i - x1] += tmp;
            costVtest[i - x1] += HCOST;
            ;
          } else {
            costVtest[i - x1] += tmp;
          }
        }
      }
      // cost for Top&Bot boundary segs (form Z with V-seg)
      grid = y2 * (xGrid - 1);
      for (j = x1; j <= x2; j++) {
        tmp =
            h_edges[grid + j].est_usage - hCapacity_lb + h_edges[grid + j].red;
        if (tmp > 0) {
          costTB[0] += tmp;
          costTBtest[0] += HCOST;
        } else {
          costTBtest[0] += tmp;
        }
      }
      grid1 = y1 * (xGrid - 1) + x1;
      grid2 = y2 * (xGrid - 1) + x1;
      for (i = 1; i <= segWidth; i++) {
        costTB[i] = costTB[i - 1];
        tmp       = h_edges[grid1 + i - 1].est_usage - hCapacity_lb +
              h_edges[grid1 + i - 1].red;
        if (tmp > 0) {
          costTB[i] += tmp;
          costTBtest[i] += HCOST;
        } else {
          costTBtest[i] += tmp;
        }
        tmp = h_edges[grid2 + i - 1].est_usage - hCapacity_lb +
              h_edges[grid2 + i - 1].red;
        if (tmp > 0) {
          costTB[i] -= tmp;
          costTBtest[i] -= HCOST;
        } else {
          costTBtest[i] -= tmp;
        }
      }
      // compute cost for all Z routing
      HVH      = true;
      bestcost = BIG_INT;
      btTEST   = BIG_INT;
      bestZ    = 0;
      for (i = 0; i <= segWidth; i++) {
        costHVH[i]     = costV[i] + costTB[i];
        costHVHtest[i] = costVtest[i] + costTBtest[i];
        if (costHVH[i] < bestcost) {
          bestcost = costHVH[i];
          btTEST   = costHVHtest[i];
          bestZ    = i + x1;
        } else if (costHVH[i] == bestcost) {
          if (costHVHtest[i] < btTEST) {
            btTEST = costHVHtest[i];
            bestZ  = i + x1;
          }
        }
      }

      if (HVH) {
        grid = y1 * (xGrid - 1);
        for (i = x1; i < bestZ; i++) {
          h_edges[grid + i].est_usage += 1;
        }
        grid = y2 * (xGrid - 1);
        for (i = bestZ; i < x2; i++) {
          h_edges[grid + i].est_usage += 1;
        }
        grid = ymin * xGrid;
        for (i = ymin; i < ymax; i++) {
          v_edges[grid + bestZ].est_usage += 1;
          grid += xGrid;
        }
        treeedge->route.HVH    = HVH;
        treeedge->route.Zpoint = bestZ;
      } else {
        printf("warning, in the maze edge, not HVH results is produced");
      }
    } // else Z route

  } // if non-degraded edge
}

// Z-route, rip-up the previous route according to the ripuptype
void newrouteZ(int netID, int threshold) {
  int ind, i, j, d, n1, n2, x1, y1, x2, y2, segWidth, segHeight, bestZ, grid,
      grid1, grid2, ymin, ymax, n1a, n2a, status1, status2;
  float tmp, bestcost, btTEST;
  bool HVH;       // the shape of Z routing (true - HVH, false - VHV)
  bool y1Smaller; // true - y1<y2, false y1>y2
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  d = sttrees[netID].deg;

  treeedges = sttrees[netID].edges;
  treenodes = sttrees[netID].nodes;

  // loop for all the tree edges (2*d-3)

  for (ind = 0; ind < 2 * d - 3; ind++) {
    treeedge = &(treeedges[ind]);

    n1 = treeedge->n1;
    n2 = treeedge->n2;
    x1 = treenodes[n1].x;
    y1 = treenodes[n1].y;
    x2 = treenodes[n2].x;
    y2 = treenodes[n2].y;

    if (sttrees[netID].edges[ind].len >
        threshold) // only route the edges with len>5
    {

      if (y1 < y2) {
        ymin = y1;
        ymax = y2;
      } else {
        ymin = y2;
        ymax = y1;
      }

      if (x1 != x2 && y1 != y2) // not H or V edge, do Z-routing
      {
        // ripup the original routing
        if (newRipupType2(treeedge, treenodes, x1, y1, x2, y2, d)) {

          n1a     = treenodes[n1].stackAlias;
          n2a     = treenodes[n2].stackAlias;
          status1 = treenodes[n1a].status;
          status2 = treenodes[n2a].status;

          treeedge->route.type = ZROUTE;

          segWidth = x2 - x1;
          if (y1 < y2) {
            ymin      = y1;
            ymax      = y2;
            y1Smaller = true;
          } else {
            ymin      = y2;
            ymax      = y1;
            y1Smaller = false;
          }
          segHeight = ymax - ymin;

          // compute the cost for all Z routing

          if (status1 == 0 || status1 == 3) {
            for (i = 0; i < segWidth; i++) {
              costHVH[i]     = 0;
              costHVHtest[i] = 0;
            }
            for (i = 0; i < segHeight; i++) {
              costVHV[i] = 0;
            }
          } else if (status1 == 2) {
            for (i = 0; i < segWidth; i++) {
              costHVH[i]     = 0;
              costHVHtest[i] = 0;
            }
            for (i = 0; i < segHeight; i++) {
              costVHV[i] = viacost;
            }
          } else {

            for (i = 0; i < segWidth; i++) {
              costHVH[i]     = viacost;
              costHVHtest[i] = viacost;
            }
            for (i = 0; i < segHeight; i++) {
              costVHV[i] = 0;
            }
          }

          if (status2 == 2) {
            for (i = 0; i < segHeight; i++) {
              costVHV[i] += viacost;
            }

          } else if (status2 == 1) {
            for (i = 0; i < segWidth; i++) {
              costHVH[i] += viacost;
              costHVHtest[i] += viacost;
            }
          }

          for (i = 0; i < segWidth; i++) {
            costV[i]  = 0;
            costTB[i] = 0;

            costVtest[i]  = 0;
            costTBtest[i] = 0;
          }
          for (i = 0; i < segHeight; i++) {
            costH[i]  = 0;
            costLR[i] = 0;
          }

          // compute the cost for all H-segs and V-segs and partial boundary seg
          // cost for V-segs
          for (i = x1; i < x2; i++) {
            grid = ymin * xGrid;
            for (j = ymin; j < ymax; j++) {
              tmp = v_edges[grid + i].est_usage - vCapacity_lb +
                    v_edges[grid + i].red;
              grid += xGrid;
              if (tmp > 0) {
                costV[i - x1] += tmp;
                costVtest[i - x1] += HCOST;
              } else {
                costVtest[i - x1] += tmp;
              }
            }
          }
          // cost for Top&Bot boundary segs (form Z with V-seg)
          grid = y2 * (xGrid - 1);
          for (j = x1; j < x2; j++) {
            tmp = h_edges[grid + j].est_usage - hCapacity_lb +
                  h_edges[grid + j].red;
            if (tmp > 0) {
              costTB[0] += tmp;
              costTBtest[0] += HCOST;
            } else {
              costTBtest[0] += tmp;
            }
          }
          grid1 = y1 * (xGrid - 1) + x1;
          grid2 = y2 * (xGrid - 1) + x1;
          for (i = 1; i < segWidth; i++) {
            costTB[i] = costTB[i - 1];
            tmp       = h_edges[grid1 + i - 1].est_usage - hCapacity_lb +
                  h_edges[grid1 + i - 1].red;
            if (tmp > 0) {
              costTB[i] += tmp;
              costTBtest[0] += HCOST;
            } else {
              costTBtest[0] += tmp;
            }
            tmp = h_edges[grid2 + i - 1].est_usage - hCapacity_lb +
                  h_edges[grid2 + i - 1].red;
            if (tmp > 0) {
              costTB[i] -= tmp;
              costTBtest[0] -= HCOST;
            } else {
              costTBtest[0] -= tmp;
            }
          }
          // cost for H-segs
          grid = ymin * (xGrid - 1);
          for (i = ymin; i < ymax; i++) {
            for (j = x1; j < x2; j++) {
              tmp = h_edges[grid + j].est_usage - hCapacity_lb +
                    h_edges[grid + j].red;
              if (tmp > 0)
                costH[i - ymin] += tmp;
            }
            grid += xGrid - 1;
          }
          // cost for Left&Right boundary segs (form Z with H-seg)
          if (y1Smaller) {
            for (j = y1; j < y2; j++) {
              tmp = v_edges[j * xGrid + x2].est_usage - vCapacity_lb +
                    v_edges[j * xGrid + x2].red;
              if (tmp > 0)
                costLR[0] += tmp;
            }
            for (i = 1; i < segHeight; i++) {
              costLR[i] = costLR[i - 1];
              grid      = (y1 + i - 1) * xGrid;
              tmp       = v_edges[grid + x1].est_usage - vCapacity_lb +
                    v_edges[grid + x1].red;
              if (tmp > 0)
                costLR[i] += tmp;
              tmp = v_edges[grid + x2].est_usage - vCapacity_lb +
                    v_edges[grid + x2].red;
              if (tmp > 0)
                costLR[i] -= tmp;
            }
          } else {
            for (j = y2; j < y1; j++) {
              tmp = v_edges[j * xGrid + x1].est_usage - vCapacity_lb;
              if (tmp > 0)
                costLR[0] += tmp;
            }
            for (i = 1; i < segHeight; i++) {
              costLR[i] = costLR[i - 1];
              grid      = (y2 + i - 1) * xGrid;
              tmp       = v_edges[grid + x2].est_usage - vCapacity_lb +
                    v_edges[grid + x2].red;
              if (tmp > 0)
                costLR[i] += tmp;
              tmp = v_edges[grid + x1].est_usage - vCapacity_lb +
                    v_edges[grid + x1].red;
              if (tmp > 0)
                costLR[i] -= tmp;
            }
          }

          // compute cost for all Z routing
          HVH      = true;
          bestcost = BIG_INT;
          btTEST   = BIG_INT;
          bestZ    = 0;
          for (i = 0; i < segWidth; i++) {
            costHVH[i] += costV[i] + costTB[i];
            if (costHVH[i] < bestcost) {
              bestcost = costHVH[i];
              btTEST   = costHVHtest[i];
              bestZ    = i + x1;
            } else if (costHVH[i] == bestcost) {
              if (costHVHtest[i] < btTEST) {
                btTEST = costHVHtest[i];
                bestZ  = i + x1;
              }
            }
          }
          for (i = 0; i < segHeight; i++) {
            costVHV[i] += costH[i] + costLR[i];
            if (costVHV[i] < bestcost) {
              bestcost = costVHV[i];
              bestZ    = i + ymin;
              HVH      = false;
            }
          }

          if (HVH) {
            if (treenodes[n1a].status < 2) {
              treenodes[n1a].status += 2;
            }
            if (treenodes[n2a].status < 2) {
              treenodes[n2a].status += 2;
            }

            treenodes[n1a].hID++;
            treenodes[n2a].hID++;

            grid = y1 * (xGrid - 1);
            for (i = x1; i < bestZ; i++) {
              h_edges[grid + i].est_usage += 1;
            }
            grid = y2 * (xGrid - 1);
            for (i = bestZ; i < x2; i++) {
              h_edges[grid + i].est_usage += 1;
            }
            grid = ymin * xGrid;
            for (i = ymin; i < ymax; i++) {
              v_edges[grid + bestZ].est_usage += 1;
              grid += xGrid;
            }
            treeedge->route.HVH    = HVH;
            treeedge->route.Zpoint = bestZ;
          } else {
            if (treenodes[n2a].status % 2 == 0) {
              treenodes[n2a].status += 1;
            }
            if (treenodes[n1a].status % 2 == 0) {
              treenodes[n1a].status += 1;
            }

            treenodes[n1a].lID++;
            treenodes[n2a].lID++;
            if (y1Smaller) {
              grid = y1 * xGrid;
              for (i = y1; i < bestZ; i++) {
                v_edges[grid + x1].est_usage += 1;
                grid += xGrid;
              }
              grid = bestZ * xGrid;
              for (i = bestZ; i < y2; i++) {
                v_edges[grid + x2].est_usage += 1;
                grid += xGrid;
              }
              grid = bestZ * (xGrid - 1);
              for (i = x1; i < x2; i++) {
                h_edges[grid + i].est_usage += 1;
              }
              treeedge->route.HVH    = HVH;
              treeedge->route.Zpoint = bestZ;
            } else {
              grid = y2 * xGrid;
              for (i = y2; i < bestZ; i++) {
                v_edges[grid + x2].est_usage += 1;
                grid += xGrid;
              }
              grid = bestZ * xGrid;
              for (i = bestZ; i < y1; i++) {
                v_edges[grid + x1].est_usage += 1;
                grid += xGrid;
              }
              grid = bestZ * (xGrid - 1);
              for (i = x1; i < x2; i++) {
                h_edges[grid + i].est_usage += 1;
              }
              treeedge->route.HVH    = HVH;
              treeedge->route.Zpoint = bestZ;
            }
          }
        } else { // if ripuped by type 2
          if (d == 2) {
            newrouteZ_edge(netID, ind);
          }
        }
      }

    } else if (d == 2 && sttrees[netID].edges[ind].len > threshold) {
      newrouteZ_edge(netID, ind);
    } // if non-degraded edge
      //        else
      //            sttrees[netID].edges[ind].route.type = NOROUTE;
  }   // loop ind
}

// ripup a tree edge according to its ripup type and Z-route it
// route all segments with L, firstTime: true, first newrouteLAll, false - not
// first
void newrouteZAll(int threshold) {
  int i;

  for (i = 0; i < numValidNets; i++) {
    newrouteZ(i, threshold); // ripup previous route and do Z-routing
  }
}

// Ripup the original route and do Monotonic routing within bounding box
void routeMonotonic(int netID, int edgeID, int threshold) {
  int i, j, cnt, x, xl, yl, xr, yr, n1, n2, x1, y1, x2, y2, grid, xGrid_1,
      ind_i, ind_j, ind_x;
  int vedge, hedge, segWidth, segHeight, curX, curY;
  int gridsX[XRANGE + YRANGE], gridsY[XRANGE + YRANGE];
  float **cost, tmp;
  bool** parent; // remember the parent of a grid on the shortest path, true -
                 // same x, false - same y
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  if (sttrees[netID].edges[edgeID].route.routelen >
      threshold) // only route the non-degraded edges (len>0)
  {
    treeedges = sttrees[netID].edges;
    treeedge  = &(treeedges[edgeID]);
    treenodes = sttrees[netID].nodes;
    n1        = treeedge->n1;
    n2        = treeedge->n2;
    x1        = treenodes[n1].x;
    y1        = treenodes[n1].y;
    x2        = treenodes[n2].x;
    y2        = treenodes[n2].y;

    if (x1 != x2 || y1 != y2) // not H or V edge, do Z-routing (if H or V edge,
                              // no need to reroute)
    {
      // ripup the original routing
      newRipup(treeedge, x1, y1, x2, y2);

      segWidth  = ADIFF(x1, x2);
      segHeight = ADIFF(y1, y2);
      if (x1 <= x2) {
        xl = x1;
        yl = y1;
        xr = x2;
        yr = y2;
      } else {
        xl = x2;
        yl = y2;
        xr = x1;
        yr = y1;
      }

      // find the best monotonic path from (x1, y1) to (x2, y2)
      cost   = (float**)malloc((segHeight + 1) * sizeof(float*));
      parent = (bool**)malloc((segHeight + 1) * sizeof(bool*));
      for (i = 0; i <= segHeight; i++) {
        cost[i]   = (float*)malloc((segWidth + 1) * sizeof(float));
        parent[i] = (bool*)malloc((segWidth + 1) * sizeof(bool));
      }

      xGrid_1 = xGrid - 1; // tmp variable to save runtime
      if (yl <= yr) {
        // initialize first column
        cost[0][0] = 0;
        grid       = yl * xGrid;
        for (j = 0; j < segHeight; j++) {
          cost[j + 1][0] =
              cost[j][0] +
              max((float)0, v_edges[grid + xl].red +
                                v_edges[grid + xl].est_usage - vCapacity_lb);
          parent[j + 1][0] = SAMEX;
          grid += xGrid;
        }
        // update other columns
        for (i = 0; i < segWidth; i++) {
          x = xl + i;
          // update the cost of a column of grids by h-edges
          grid = yl * xGrid_1;
          for (j = 0; j <= segHeight; j++) {
            tmp              = max((float)0, h_edges[grid + x].red +
                                    h_edges[grid + x].est_usage - hCapacity_lb);
            cost[j][i + 1]   = cost[j][i] + tmp;
            parent[j][i + 1] = SAMEY;
            grid += xGrid - 1;
          }
          // update the cost of a column of grids by v-edges
          grid  = yl * xGrid;
          ind_x = x + 1;
          ind_i = i + 1;
          for (j = 0; j < segHeight; j++) {
            ind_j = j + 1;
            tmp   = cost[j][ind_i] +
                  max((float)0, v_edges[grid + ind_x].red +
                                    v_edges[grid + ind_x].est_usage -
                                    vCapacity_lb);
            if (cost[ind_j][ind_i] > tmp) {
              cost[ind_j][ind_i]   = tmp;
              parent[ind_j][ind_i] = SAMEX;
            }
            grid += xGrid;
          }
        }

        // store the shortest path and update the usage
        curX = xr;
        curY = yr;
        cnt  = 0;

        while (curX != xl || curY != yl) {
          // printf("xl is %d, yl is %d, curX is %d, curY is
          // %d\n",xl,yl,curX,curY); printf("%d\n", sttrees[netID].deg);
          gridsX[cnt] = curX;
          gridsY[cnt] = curY;
          cnt++;
          if (parent[curY - yl][curX - xl] == SAMEX) {
            curY--;
            vedge = curY * xGrid + curX;
            v_edges[vedge].est_usage += 1;
          } else {
            curX--;
            hedge = curY * (xGrid - 1) + curX;
            h_edges[hedge].est_usage += 1;
          }
        }

        gridsX[cnt] = xl;
        gridsY[cnt] = yl;
        cnt++;

      } // yl<=yr

      else // yl>yr
      {
        // initialize first column
        cost[segHeight][0] = 0;
        grid               = (yl - 1) * xGrid;
        for (j = segHeight - 1; j >= 0; j--) {
          cost[j][0] =
              cost[j + 1][0] +
              max((float)0, v_edges[grid + xl].red +
                                v_edges[grid + xl].est_usage - vCapacity_lb);
          parent[j][0] = SAMEX;
          grid -= xGrid;
        }
        // update other columns
        for (i = 0; i < segWidth; i++) {
          x = xl + i;
          // update the cost of a column of grids by h-edges
          grid  = yl * (xGrid - 1);
          ind_i = i + 1;
          for (j = segHeight; j >= 0; j--) {
            tmp              = max((float)0, h_edges[grid + x].red +
                                    h_edges[grid + x].est_usage - hCapacity_lb);
            cost[j][ind_i]   = cost[j][i] + tmp;
            parent[j][ind_i] = SAMEY;
            grid -= xGrid - 1;
          }
          // update the cost of a column of grids by v-edges
          grid  = (yl - 1) * xGrid;
          ind_x = x + 1;
          for (j = segHeight - 1; j >= 0; j--) {
            tmp = cost[j + 1][ind_i] +
                  max((float)0, v_edges[grid + ind_x].red +
                                    v_edges[grid + ind_x].est_usage -
                                    vCapacity_lb);
            if (cost[j][ind_i] > tmp) {
              cost[j][ind_i]   = tmp;
              parent[j][ind_i] = SAMEX;
            }
            grid -= xGrid;
          }
        }

        // store the shortest path and update the usage
        curX = xr;
        curY = yr;
        cnt  = 0;
        while (curX != xl || curY != yl) {
          gridsX[cnt] = curX;
          gridsY[cnt] = curY;
          cnt++;
          if (parent[curY - yr][curX - xl] == SAMEX) {
            vedge = curY * xGrid + curX;
            v_edges[vedge].est_usage += 1;
            curY++;
          } else {
            curX--;
            hedge = curY * (xGrid - 1) + curX;
            h_edges[hedge].est_usage += 1;
          }
        }
        gridsX[cnt] = xl;
        gridsY[cnt] = yl;
        cnt++;

      } // yl>yr
      treeedge->route.routelen = cnt - 1;

      treeedge->route.gridsX =
          (short*)realloc(treeedge->route.gridsX, cnt * sizeof(short));
      treeedge->route.gridsY =
          (short*)realloc(treeedge->route.gridsY, cnt * sizeof(short));
      if (x1 != gridsX[0] ||
          y1 != gridsY[0]) // gridsX[] and gridsY[] store the path from n2 to n1
      {
        cnt = 0;
        for (i = treeedge->route.routelen; i >= 0; i--) {
          treeedge->route.gridsX[cnt] = gridsX[i];
          treeedge->route.gridsY[cnt] = gridsY[i];
          cnt++;
        }
      } else // gridsX[] and gridsY[] store the path from n1 to n2
      {
        for (i = 0; i <= treeedge->route.routelen; i++) {
          treeedge->route.gridsX[i] = gridsX[i];
          treeedge->route.gridsY[i] = gridsY[i];
        }
      }

      for (i = 0; i <= segHeight; i++) {
        free(cost[i]);
        free(parent[i]);
      }
      free(cost);
      free(parent);

    } // if(x1!=x2 || y1!=y2)
  }   // non-degraded edge
}

void routeMonotonicAll(int threshold) {
  int netID, edgeID;

  for (netID = 0; netID < numValidNets; netID++) {
    for (edgeID = 0; edgeID < sttrees[netID].deg * 2 - 3; edgeID++) {
      routeMonotonic(
          netID, edgeID,
          threshold); // ripup previous route and do Monotonic routing
    }
  }
  printf("MonotonicAll OK\n");
}

void spiralRoute(int netID, int edgeID) {
  int j, n1, n2, x1, y1, x2, y2, grid, grid1, n1a, n2a;
  float costL1 = 0, costL2 = 0, tmp;
  int ymin, ymax;
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  treeedges = sttrees[netID].edges;
  treenodes = sttrees[netID].nodes;

  // loop for all the tree edges (2*d-3)

  treeedge = &(treeedges[edgeID]);
  if (treeedge->len > 0) // only route the non-degraded edges (len>0)
  {

    n1 = treeedge->n1;
    n2 = treeedge->n2;
    x1 = treenodes[n1].x;
    y1 = treenodes[n1].y;
    x2 = treenodes[n2].x;
    y2 = treenodes[n2].y;

    n1a = treenodes[n1].stackAlias;
    n2a = treenodes[n2].stackAlias;

    if (y1 < y2) {
      ymin = y1;
      ymax = y2;
    } else {
      ymin = y2;
      ymax = y1;
    }

    // ripup the original routing

    treeedge->route.type = LROUTE;
    if (x1 == x2) // V-routing
    {
      for (j = ymin; j < ymax; j++)
        v_edges[j * xGrid + x1].est_usage++;
      treeedge->route.xFirst = false;
      if (treenodes[n1].status % 2 == 0) {
        treenodes[n1].status += 1;
      }
      if (treenodes[n2].status % 2 == 0) {
        treenodes[n2].status += 1;
      }

      if (treenodes[n1a].status % 2 == 0) {
        treenodes[n1a].status += 1;
      }
      if (treenodes[n2a].status % 2 == 0) {
        treenodes[n2a].status += 1;
      }
    } else if (y1 == y2) // H-routing
    {
      for (j = x1; j < x2; j++)
        h_edges[y1 * (xGrid - 1) + j].est_usage++;
      treeedge->route.xFirst = true;
      if (treenodes[n2].status < 2) {
        treenodes[n2].status += 2;
      }
      if (treenodes[n1].status < 2) {
        treenodes[n1].status += 2;
      }

      if (treenodes[n2a].status < 2) {
        treenodes[n2a].status += 2;
      }
      if (treenodes[n1a].status < 2) {
        treenodes[n1a].status += 2;
      }
    } else // L-routing
    {

      if (treenodes[n1].status == 0 || treenodes[n1].status == 3) {
        costL1 = costL2 = 0;
      } else if (treenodes[n1].status == 2) {
        costL1 = viacost;
        costL2 = 0;
      } else if (treenodes[n1].status == 1) {

        costL1 = 0;
        costL2 = viacost;
      } else {
        printf("wrong node status %d", treenodes[n1].status);
      }
      if (treenodes[n2].status == 2) {
        costL2 += viacost;
      } else if (treenodes[n2].status == 1) {
        costL1 += viacost;
      }

      for (j = ymin; j < ymax; j++) {
        grid = j * xGrid;
        tmp  = v_edges[grid + x1].est_usage - vCapacity_lb +
              v_edges[grid + x1].red;
        if (tmp > 0)
          costL1 += tmp;
        tmp = v_edges[grid + x2].est_usage - vCapacity_lb +
              v_edges[grid + x2].red;
        if (tmp > 0)
          costL2 += tmp;
        // costL1 += simpleCost (v_edges[grid+x1].est_usage,
        // v_edges[grid+x1].cap); costL2 += simpleCost (
        // v_edges[grid+x2].est_usage,  v_edges[grid+x2].cap);
      }
      grid  = y2 * (xGrid - 1);
      grid1 = y1 * (xGrid - 1);
      for (j = x1; j < x2; j++) {
        tmp =
            h_edges[grid + j].est_usage - hCapacity_lb + h_edges[grid + j].red;
        if (tmp > 0)
          costL1 += tmp;
        tmp = h_edges[grid1 + j].est_usage - hCapacity_lb +
              h_edges[grid1 + j].red;
        if (tmp > 0)
          costL2 += tmp;
        // costL1 += simpleCost (h_edges[grid+j].est_usage,
        // h_edges[grid+j].cap); costL2 += simpleCost
        // (h_edges[grid1+j].est_usage, h_edges[grid1+j].cap);
      }

      if (costL1 < costL2) {
        if (treenodes[n1].status % 2 == 0) {
          treenodes[n1].status += 1;
        }
        if (treenodes[n2].status < 2) {
          treenodes[n2].status += 2;
        }

        if (treenodes[n1a].status % 2 == 0) {
          treenodes[n1a].status += 1;
        }
        if (treenodes[n2a].status < 2) {
          treenodes[n2a].status += 2;
        }
        treenodes[n2a].hID++;
        treenodes[n1a].lID++;

        // two parts (x1, y1)-(x1, y2) and (x1, y2)-(x2, y2)
        for (j = ymin; j < ymax; j++) {
          v_edges[j * xGrid + x1].est_usage += 1;
        }
        grid = y2 * (xGrid - 1);
        for (j = x1; j < x2; j++) {
          h_edges[grid + j].est_usage += 1;
        }
        treeedge->route.xFirst = false;
      } // if costL1<costL2
      else {
        if (treenodes[n2].status % 2 == 0) {
          treenodes[n2].status += 1;
        }
        if (treenodes[n1].status < 2) {
          treenodes[n1].status += 2;
        }

        if (treenodes[n2a].status % 2 == 0) {
          treenodes[n2a].status += 1;
        }

        if (treenodes[n1a].status < 2) {
          treenodes[n1a].status += 2;
        }
        treenodes[n1a].hID++;
        treenodes[n2a].lID++;

        // two parts (x1, y1)-(x2, y1) and (x2, y1)-(x2, y2)
        grid = y1 * (xGrid - 1);
        for (j = x1; j < x2; j++) {
          h_edges[grid + j].est_usage += 1;
        }
        for (j = ymin; j < ymax; j++) {
          v_edges[j * xGrid + x2].est_usage += 1;
        }
        treeedge->route.xFirst = true;
      }

    } // else L-routing
  }   // if non-degraded edge
  else
    sttrees[netID].edges[edgeID].route.type = NOROUTE;
}

void spiralRouteAll() {
  int netID, d, k, edgeID, nodeID, deg, numpoints, n1, n2;
  int na;
  bool redundant;
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;
  int quehead, quetail;
  int edgeQueue[5000];

  for (netID = 0; netID < numValidNets; netID++) {
    treeedges = sttrees[netID].edges;
    treenodes = sttrees[netID].nodes;
    deg       = sttrees[netID].deg;

    numpoints = 0;

    for (d = 0; d < 2 * deg - 2; d++) {
      treenodes[d].topL = -1;
      treenodes[d].botL = MAXLAYER;
      // treenodes[d].l = 0;
      treenodes[d].assigned   = false;
      treenodes[d].stackAlias = d;
      treenodes[d].conCNT     = 0;
      treenodes[d].hID        = 0;
      treenodes[d].lID        = 0;
      treenodes[d].status     = 0;

      if (d < deg) {
        treenodes[d].botL = treenodes[d].topL = 0;
        // treenodes[d].l = 0;
        treenodes[d].assigned = true;
        treenodes[d].status   = 2;

        xcor[numpoints] = treenodes[d].x;
        ycor[numpoints] = treenodes[d].y;
        dcor[numpoints] = d;
        numpoints++;
      } else {
        redundant = false;
        for (k = 0; k < numpoints; k++) {
          if ((treenodes[d].x == xcor[k]) && (treenodes[d].y == ycor[k])) {
            treenodes[d].stackAlias = dcor[k];
            redundant               = true;
            break;
          }
        }
        if (!redundant) {
          xcor[numpoints] = treenodes[d].x;
          ycor[numpoints] = treenodes[d].y;
          dcor[numpoints] = d;
          numpoints++;
        }
      }
    }
  }

  for (netID = 0; netID < numValidNets; netID++) {
    treeedges = sttrees[netID].edges;
    treenodes = sttrees[netID].nodes;
    deg       = sttrees[netID].deg;

    for (edgeID = 0; edgeID < 2 * deg - 3; edgeID++) {
      treeedge = &(treeedges[edgeID]);
      if (treeedge->len > 0) {

        n1 = treeedge->n1;
        n2 = treeedge->n2;

        treeedge->n1a = treenodes[n1].stackAlias;
        treenodes[treeedge->n1a].eID[treenodes[treeedge->n1a].conCNT] = edgeID;
        treenodes[treeedge->n1a].conCNT++;

        treeedge->n2a = treenodes[n2].stackAlias;
        treenodes[treeedge->n2a].eID[treenodes[treeedge->n2a].conCNT] = edgeID;
        (treenodes[treeedge->n2a].conCNT)++;
        treeedges[edgeID].assigned = false;
      } else {
        treeedges[edgeID].assigned = true;
      }
    }
  }

  for (netID = 0; netID < numValidNets; netID++) {

    newRipupNet(netID);

    treeedges = sttrees[netID].edges;
    treenodes = sttrees[netID].nodes;
    deg       = sttrees[netID].deg;
    quehead = quetail = 0;

    for (nodeID = 0; nodeID < deg; nodeID++) {
      treenodes[nodeID].assigned = true;
      for (k = 0; k < treenodes[nodeID].conCNT; k++) {

        edgeID = treenodes[nodeID].eID[k];

        if (treeedges[edgeID].assigned == false) {
          edgeQueue[quetail]         = edgeID;
          treeedges[edgeID].assigned = true;
          quetail++;
        }
      }
    }

    while (quehead != quetail) {
      edgeID   = edgeQueue[quehead];
      treeedge = &(treeedges[edgeID]);
      if (treenodes[treeedge->n1a].assigned) {
        spiralRoute(netID, edgeID);
        treeedge->assigned = true;
        if (!treenodes[treeedge->n2a].assigned) {
          for (k = 0; k < treenodes[treeedge->n2a].conCNT; k++) {
            edgeID = treenodes[treeedge->n2a].eID[k];
            if (!treeedges[edgeID].assigned) {
              edgeQueue[quetail]         = edgeID;
              treeedges[edgeID].assigned = true;
              quetail++;
            }
          }
          treenodes[treeedge->n2a].assigned = true;
        }
      } else {
        spiralRoute(netID, edgeID);
        treeedge->assigned = true;
        if (!treenodes[treeedge->n1a].assigned) {
          for (k = 0; k < treenodes[treeedge->n1a].conCNT; k++) {
            edgeID = treenodes[treeedge->n1a].eID[k];
            if (!treeedges[edgeID].assigned) {
              edgeQueue[quetail]         = edgeID;
              treeedges[edgeID].assigned = true;
              quetail++;
            }
          }
          treenodes[treeedge->n1a].assigned = true;
        }
      }
      quehead++;
    }
  }

  for (netID = 0; netID < numValidNets; netID++) {
    treenodes = sttrees[netID].nodes;
    deg       = sttrees[netID].deg;

    for (d = 0; d < 2 * deg - 2; d++) {
      na = treenodes[d].stackAlias;

      treenodes[d].status = treenodes[na].status;
    }
  }
}

void routeLVEnew(int netID, int edgeID, int threshold, int enlarge) {
  int i, j, cnt, xmin, xmax, ymin, ymax, n1, n2, x1, y1, x2, y2, grid, xGrid_1,
      deg, yminorig, ymaxorig;
  int vedge, hedge, bestp1x = 0, bestp1y = 0;
  int gridsX[XRANGE + YRANGE], gridsY[XRANGE + YRANGE];
  float tmp1, tmp2, tmp3, tmp4, tmp, best;
  bool LH1, LH2, BL1 = false, BL2 = false;
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  if (sttrees[netID].edges[edgeID].len >
      threshold) // only route the non-degraded edges (len>0)
  {
    treeedges = sttrees[netID].edges;
    treeedge  = &(treeedges[edgeID]);
    treenodes = sttrees[netID].nodes;
    n1        = treeedge->n1;
    n2        = treeedge->n2;
    x1        = treenodes[n1].x;
    y1        = treenodes[n1].y;
    x2        = treenodes[n2].x;
    y2        = treenodes[n2].y;

    // ripup the original routing
    if (newRipupCheck(treeedge, threshold, netID, edgeID)) {

      deg  = sttrees[netID].deg;
      xmin = max(x1 - enlarge, 0);
      xmax = min(xGrid - 1, x2 + enlarge);

      if (y1 < y2) {
        ymin     = max(y1 - enlarge, 0);
        ymax     = min(yGrid - 1, y2 + enlarge);
        yminorig = y1;
        ymaxorig = y2;
      } else {
        ymin     = max(y2 - enlarge, 0);
        ymax     = min(yGrid - 1, y1 + enlarge);
        yminorig = y2;
        ymaxorig = y1;
      }

      if (deg > 2) {
        for (j = 0; j < deg * 2 - 2; j++) {
          if (treenodes[j].x < x1) {
            xmin = x1;
          }
          if (treenodes[j].x > x2) {
            xmax = x2;
          }
          if (treenodes[j].y < yminorig) {
            ymin = yminorig;
          }
          if (treenodes[j].y > ymaxorig) {
            ymax = ymaxorig;
          }
        }
      }

      xGrid_1 = xGrid - 1; // tmp variable to save runtime

      for (j = ymin; j <= ymax; j++) {
        d1[j][xmin] = 0;
      }
      // update other columns
      for (i = xmin; i <= xmax; i++) {
        d2[ymin][i] = 0;
      }

      for (j = ymin; j <= ymax; j++) {
        grid = j * xGrid_1 + xmin;
        for (i = xmin; i < xmax; i++) {
          tmp          = h_costTable[h_edges[grid].red + h_edges[grid].usage];
          d1[j][i + 1] = d1[j][i] + tmp;
          grid++;
        }
        // update the cost of a column of grids by v-edges
      }

      for (j = ymin; j < ymax; j++) {
        // update the cost of a column of grids by h-edges
        grid = j * xGrid + xmin;
        for (i = xmin; i <= xmax; i++) {
          tmp          = h_costTable[v_edges[grid].red + v_edges[grid].usage];
          d2[j + 1][i] = d2[j][i] + tmp;
          grid++;
        }
        // update the cost of a column of grids by v-edges
      }

      best = BIG_INT;

      for (j = ymin; j <= ymax; j++) {
        for (i = xmin; i <= xmax; i++) {

          tmp1 = ADIFF(d2[j][x1], d2[y1][x1]) +
                 ADIFF(d1[j][i], d1[j][x1]); // yfirst for point 1
          tmp2 = ADIFF(d2[j][i], d2[y1][i]) + ADIFF(d1[y1][i], d1[y1][x1]);
          tmp3 = ADIFF(d2[y2][i], d2[j][i]) + ADIFF(d1[y2][i], d1[y2][x2]);
          tmp4 = ADIFF(d2[y2][x2], d2[j][x2]) +
                 ADIFF(d1[j][x2], d1[j][i]); // xifrst for mid point

          tmp = tmp1 + tmp4;
          LH1 = false;
          LH2 = true;

          if (tmp2 + tmp3 < tmp) {
            tmp = tmp2 + tmp3;
            LH1 = true;
            LH2 = false;
          }

          if (tmp1 + tmp3 + viacost < tmp) {
            LH1 = false;
            LH2 = false;
            tmp = tmp1 + tmp3 + viacost;
          }

          if (tmp2 + tmp4 + viacost < tmp) {
            LH1 = true;
            LH2 = true;
            tmp = tmp2 + tmp4 + viacost;
          }

          if (tmp < best) {
            bestp1x = i;
            bestp1y = j;
            BL1     = LH1;
            BL2     = LH2;
            best    = tmp;
          }
        }
      }
      cnt = 0;

      if (BL1) {
        if (bestp1x > x1) {
          for (i = x1; i < bestp1x; i++) {
            gridsX[cnt] = i;
            gridsY[cnt] = y1;
            hedge       = y1 * xGrid_1 + i;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        } else {
          for (i = x1; i > bestp1x; i--) {
            gridsX[cnt] = i;
            gridsY[cnt] = y1;
            hedge       = y1 * xGrid_1 + i - 1;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        }
        if (bestp1y > y1) {
          for (i = y1; i < bestp1y; i++) {
            gridsX[cnt] = bestp1x;
            gridsY[cnt] = i;
            cnt++;
            vedge = i * xGrid + bestp1x;
            v_edges[vedge].usage += 1;
          }
        } else {
          for (i = y1; i > bestp1y; i--) {
            gridsX[cnt] = bestp1x;
            gridsY[cnt] = i;
            cnt++;
            vedge = (i - 1) * xGrid + bestp1x;
            v_edges[vedge].usage += 1;
          }
        }
      } else {
        if (bestp1y > y1) {
          for (i = y1; i < bestp1y; i++) {
            gridsX[cnt] = x1;
            gridsY[cnt] = i;
            cnt++;
            vedge = i * xGrid + x1;
            v_edges[vedge].usage += 1;
          }
        } else {
          for (i = y1; i > bestp1y; i--) {
            gridsX[cnt] = x1;
            gridsY[cnt] = i;
            cnt++;
            vedge = (i - 1) * xGrid + x1;
            v_edges[vedge].usage += 1;
          }
        }
        if (bestp1x > x1) {
          for (i = x1; i < bestp1x; i++) {
            gridsX[cnt] = i;
            gridsY[cnt] = bestp1y;
            hedge       = bestp1y * xGrid_1 + i;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        } else {
          for (i = x1; i > bestp1x; i--) {
            gridsX[cnt] = i;
            gridsY[cnt] = bestp1y;
            hedge       = bestp1y * xGrid_1 + i - 1;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        }
      }

      if (BL2) {
        if (bestp1x < x2) {
          for (i = bestp1x; i < x2; i++) {
            gridsX[cnt] = i;
            gridsY[cnt] = bestp1y;
            hedge       = bestp1y * xGrid_1 + i;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        } else {
          for (i = bestp1x; i > x2; i--) {
            gridsX[cnt] = i;
            gridsY[cnt] = bestp1y;
            hedge       = bestp1y * xGrid_1 + i - 1;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        }

        if (y2 > bestp1y) {
          for (i = bestp1y; i < y2; i++) {
            gridsX[cnt] = x2;
            gridsY[cnt] = i;
            cnt++;
            vedge = i * xGrid + x2;
            v_edges[vedge].usage += 1;
          }
        } else {
          for (i = bestp1y; i > y2; i--) {
            gridsX[cnt] = x2;
            gridsY[cnt] = i;
            cnt++;
            vedge = (i - 1) * xGrid + x2;
            v_edges[vedge].usage += 1;
          }
        }
      } else {

        if (y2 > bestp1y) {
          for (i = bestp1y; i < y2; i++) {
            gridsX[cnt] = bestp1x;
            gridsY[cnt] = i;
            cnt++;
            vedge = i * xGrid + bestp1x;
            v_edges[vedge].usage += 1;
          }
        } else {
          for (i = bestp1y; i > y2; i--) {
            gridsX[cnt] = bestp1x;
            gridsY[cnt] = i;
            cnt++;
            vedge = (i - 1) * xGrid + bestp1x;
            v_edges[vedge].usage += 1;
          }
        }
        if (x2 > bestp1x) {
          for (i = bestp1x; i < x2; i++) {
            gridsX[cnt] = i;
            gridsY[cnt] = y2;
            hedge       = y2 * xGrid_1 + i;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        } else {
          for (i = bestp1x; i > x2; i--) {
            gridsX[cnt] = i;
            gridsY[cnt] = y2;
            hedge       = y2 * xGrid_1 + i - 1;
            h_edges[hedge].usage += 1;
            cnt++;
          }
        }
      }

      gridsX[cnt] = x2;
      gridsY[cnt] = y2;
      cnt++;

      treeedge->route.routelen = cnt - 1;
      free(treeedge->route.gridsX);
      free(treeedge->route.gridsY);

      treeedge->route.gridsX = (short*)calloc(cnt, sizeof(short));
      treeedge->route.gridsY = (short*)calloc(cnt, sizeof(short));

      for (i = 0; i < cnt; i++) {
        treeedge->route.gridsX[i] = gridsX[i];
        treeedge->route.gridsY[i] = gridsY[i];
      }

    } // if(x1!=x2 || y1!=y2)
  }   // non-degraded edge
}

void SPRoute::routeLVAll(int threshold, int expand) {
  int netID, edgeID, numEdges, i, forange;

  if(verbose_ > none)
    printf("%d threshold, %d expand\n", threshold, expand);

  h_costTable = (float*)calloc(10 * hCapacity, sizeof(float));

  forange = 10 * hCapacity;
  for (i = 0; i < forange; i++) {
    h_costTable[i] =
        costheight / (exp((float)(hCapacity - i) * LOGIS_COF) + 1) +
        1; // /hCapacity*30));
  }

  for (netID = 0; netID < numValidNets; netID++) {
    numEdges = 2 * sttrees[netID].deg - 3;
    for (edgeID = 0; edgeID < numEdges; edgeID++) {
      routeLVEnew(netID, edgeID, threshold,
                  expand); // ripup previous route and do Monotonic routing
    }
  }
  free(h_costTable);
  // printf("LV routing OK\n");
}

void newrouteLInMaze(int netID) {
  int i, j, d, n1, n2, x1, y1, x2, y2, grid, grid1;
  int costL1, costL2, tmp;
  int ymin, ymax;
  TreeEdge *treeedges, *treeedge;
  TreeNode* treenodes;

  d         = sttrees[netID].deg;
  treeedges = sttrees[netID].edges;
  treenodes = sttrees[netID].nodes;

  // loop for all the tree edges (2*d-3)
  for (i = 0; i < 2 * d - 3; i++) {
    if (sttrees[netID].edges[i].len >
        0) // only route the non-degraded edges (len>0)
    {

      treeedge = &(treeedges[i]);

      n1 = treeedge->n1;
      n2 = treeedge->n2;
      x1 = treenodes[n1].x;
      y1 = treenodes[n1].y;
      x2 = treenodes[n2].x;
      y2 = treenodes[n2].y;

      if (y1 < y2) {
        ymin = y1;
        ymax = y2;
      } else {
        ymin = y2;
        ymax = y1;
      }

      treeedge->route.type = LROUTE;
      if (x1 == x2) // V-routing
      {
        for (j = ymin; j < ymax; j++)
          v_edges[j * xGrid + x1].usage++;
        treeedge->route.xFirst = false;

      } else if (y1 == y2) // H-routing
      {
        for (j = x1; j < x2; j++)
          h_edges[y1 * (xGrid - 1) + j].usage++;
        treeedge->route.xFirst = true;

      } else // L-routing
      {

        costL1 = costL2 = 0;

        for (j = ymin; j < ymax; j++) {
          grid = j * xGrid;
          tmp =
              v_edges[grid + x1].usage - vCapacity_lb + v_edges[grid + x1].red;
          if (tmp > 0)
            costL1 += tmp;
          tmp =
              v_edges[grid + x2].usage - vCapacity_lb + v_edges[grid + x2].red;
          if (tmp > 0)
            costL2 += tmp;
          // costL1 += simpleCost (v_edges[grid+x1].est_usage,
          // v_edges[grid+x1].cap); costL2 += simpleCost (
          // v_edges[grid+x2].est_usage,  v_edges[grid+x2].cap);
        }
        grid  = y2 * (xGrid - 1);
        grid1 = y1 * (xGrid - 1);
        for (j = x1; j < x2; j++) {
          tmp = h_edges[grid + j].usage - hCapacity_lb + h_edges[grid + j].red;
          if (tmp > 0)
            costL1 += tmp;
          tmp =
              h_edges[grid1 + j].usage - hCapacity_lb + h_edges[grid1 + j].red;
          if (tmp > 0)
            costL2 += tmp;
        }

        if (costL1 < costL2) {

          // two parts (x1, y1)-(x1, y2) and (x1, y2)-(x2, y2)
          for (j = ymin; j < ymax; j++) {
            v_edges[j * xGrid + x1].usage += 1;
          }
          grid = y2 * (xGrid - 1);
          for (j = x1; j < x2; j++) {
            h_edges[grid + j].usage += 1;
          }
          treeedge->route.xFirst = false;
        } // if costL1<costL2
        else {

          // two parts (x1, y1)-(x2, y1) and (x2, y1)-(x2, y2)
          grid = y1 * (xGrid - 1);
          for (j = x1; j < x2; j++) {
            h_edges[grid + j].usage += 1;
          }
          for (j = ymin; j < ymax; j++) {
            v_edges[j * xGrid + x2].usage += 1;
          }
          treeedge->route.xFirst = true;
        }

      } // else L-routing
    }   // if non-degraded edge
    else
      sttrees[netID].edges[i].route.type = NOROUTE;
  } // loop i
}

} //namespace sproute

