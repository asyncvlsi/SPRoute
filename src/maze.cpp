#include "sproute.h"
#include "maze.h"
#include "global_variable_extern.h"

namespace sproute {

bool checkIfDone(TreeEdge* treeedge) {
    short *gridsX, *gridsY;
    int grid, ymin, xmin;
    
    if (treeedge->len == 0) {
        return true;
    }
    
    gridsX = treeedge->route.gridsX;
    gridsY = treeedge->route.gridsY;

    for (int i = 0; i < treeedge->route.routelen; i++) {
	    if (gridsX[i] == gridsX[i + 1]) // a vertical edge
        {   
            ymin = min(gridsY[i], gridsY[i + 1]);
            grid = ymin * xGrid + gridsX[i];
        //
        //if( v_edges[grid].usage + v_edges[grid].red > vCapacity - ripup_threshold)
            if(v_edges[grid].usage > v_edges[grid].cap)	
                return false;
   	    } 
		else if (gridsY[i] == gridsY[i + 1]) {// a horizontal edge 
            xmin = min(gridsX[i], gridsX[i + 1]);
            grid = gridsY[i] * (xGrid - 1) + xmin;
        //   if(h_edges[grid].usage + h_edges[grid].red > hCapacity - ripup_threshold)
            if(h_edges[grid].usage > h_edges[grid].cap) 
                return false;
        }
        else {
            cout << "error in checkIfDone" << endl;
            exit(1);
        }
    }
	return true;
}

void convertToMazerouteNet(int netID) {
  short *gridsX, *gridsY;
  int i, edgeID, edgelength;
  int n1, n2, x1, y1, x2, y2;
  int cnt, Zpoint;
  TreeEdge* treeedge;
  TreeNode* treenodes;

  treenodes = sttrees[netID].nodes;
  for (edgeID = 0; edgeID < 2 * sttrees[netID].deg - 3; edgeID++) {
    treeedge               = &(sttrees[netID].edges[edgeID]);
    edgelength             = treeedge->len;
    n1                     = treeedge->n1;
    n2                     = treeedge->n2;
    x1                     = treenodes[n1].x;
    y1                     = treenodes[n1].y;
    x2                     = treenodes[n2].x;
    y2                     = treenodes[n2].y;
    treeedge->route.gridsX = (short*)calloc((edgelength + 1), sizeof(short));
    treeedge->route.gridsY = (short*)calloc((edgelength + 1), sizeof(short));
    gridsX                 = treeedge->route.gridsX;
    gridsY                 = treeedge->route.gridsY;
    treeedge->len          = ADIFF(x1, x2) + ADIFF(y1, y2);

    cnt = 0;
    if (treeedge->route.type == NOROUTE) {
      gridsX[0]                = x1;
      gridsY[0]                = y1;
      treeedge->route.type     = MAZEROUTE;
      treeedge->route.routelen = 0;
      treeedge->len            = 0;
      cnt++;
    } else if (treeedge->route.type == LROUTE) {
      if (treeedge->route.xFirst) // horizontal first
      {
        for (i = x1; i <= x2; i++) {
          gridsX[cnt] = i;
          gridsY[cnt] = y1;
          cnt++;
        }
        if (y1 <= y2) {
          for (i = y1 + 1; i <= y2; i++) {
            gridsX[cnt] = x2;
            gridsY[cnt] = i;
            cnt++;
          }
        } else {
          for (i = y1 - 1; i >= y2; i--) {
            gridsX[cnt] = x2;
            gridsY[cnt] = i;
            cnt++;
          }
        }
      } else // vertical first
      {
        if (y1 <= y2) {
          for (i = y1; i <= y2; i++) {
            gridsX[cnt] = x1;
            gridsY[cnt] = i;
            cnt++;
          }
        } else {
          for (i = y1; i >= y2; i--) {
            gridsX[cnt] = x1;
            gridsY[cnt] = i;
            cnt++;
          }
        }
        for (i = x1 + 1; i <= x2; i++) {
          gridsX[cnt] = i;
          gridsY[cnt] = y2;
          cnt++;
        }
      }
    } else if (treeedge->route.type == ZROUTE) {
      Zpoint = treeedge->route.Zpoint;
      if (treeedge->route.HVH) // HVH
      {
        for (i = x1; i < Zpoint; i++) {
          gridsX[cnt] = i;
          gridsY[cnt] = y1;
          cnt++;
        }
        if (y1 <= y2) {
          for (i = y1; i <= y2; i++) {
            gridsX[cnt] = Zpoint;
            gridsY[cnt] = i;
            cnt++;
          }
        } else {
          for (i = y1; i >= y2; i--) {
            gridsX[cnt] = Zpoint;
            gridsY[cnt] = i;
            cnt++;
          }
        }
        for (i = Zpoint + 1; i <= x2; i++) {
          gridsX[cnt] = i;
          gridsY[cnt] = y2;
          cnt++;
        }
      } else // VHV
      {
        if (y1 <= y2) {
          for (i = y1; i < Zpoint; i++) {
            gridsX[cnt] = x1;
            gridsY[cnt] = i;
            cnt++;
          }
          for (i = x1; i <= x2; i++) {
            gridsX[cnt] = i;
            gridsY[cnt] = Zpoint;
            cnt++;
          }
          for (i = Zpoint + 1; i <= y2; i++) {
            gridsX[cnt] = x2;
            gridsY[cnt] = i;
            cnt++;
          }
        } else {
          for (i = y1; i > Zpoint; i--) {
            gridsX[cnt] = x1;
            gridsY[cnt] = i;
            cnt++;
          }
          for (i = x1; i <= x2; i++) {
            gridsX[cnt] = i;
            gridsY[cnt] = Zpoint;
            cnt++;
          }
          for (i = Zpoint - 1; i >= y2; i--) {
            gridsX[cnt] = x2;
            gridsY[cnt] = i;
            cnt++;
          }
        }
      }
    }

    treeedge->route.type     = MAZEROUTE;
    treeedge->route.routelen = edgelength;

  } // loop for all the edges
}

void convertToMazeroute() {
  int i, j, netID;

  for (netID = 0; netID < numValidNets; netID++) {
    convertToMazerouteNet(netID);
  }

  for (i = 0; i < yGrid; i++) {
    for (j = 0; j < xGrid - 1; j++) {
      int grid            = i * (xGrid - 1) + j;
      h_edges[grid].usage = h_edges[grid].est_usage;
    }
  }
  //    fprintf(fpv, "\nVertical Congestion\n");
  for (i = 0; i < yGrid - 1; i++) {
    for (j = 0; j < xGrid; j++) {
      int grid            = i * xGrid + j;
      v_edges[grid].usage = v_edges[grid].est_usage;
    }
  }
}

// non recursive version of heapify
void heapify(float** array, int heapSize, int i) {
  int l, r, smallest;
  float* tmp;
  bool STOP = false;

  tmp = array[i];
  do {

    l = LEFT(i);
    r = RIGHT(i);

    if (l < heapSize && *(array[l]) < *tmp) {
      smallest = l;
      if (r < heapSize && *(array[r]) < *(array[l]))
        smallest = r;
    } else {
      smallest = i;
      if (r < heapSize && *(array[r]) < *tmp)
        smallest = r;
    }
    if (smallest != i) {
      array[i] = array[smallest];
      i        = smallest;
    } else {
      array[i] = tmp;
      STOP     = true;
    }
  } while (!STOP);
}

// build heap for an list of grid
/*void buildHeap(float **array, int arrayLen)
{
    int i;

    for (i=arrayLen/2-1; i>=0; i--)
        heapify(array, arrayLen, i);
}*/

void updateHeap(float** array, int i) {
  int parent;
  float* tmpi;

  tmpi = array[i];
  while (i > 0 && *(array[PARENT(i)]) > *tmpi) {
    parent   = PARENT(i);
    array[i] = array[parent];
    i        = parent;
  }
  array[i] = tmpi;
}

// extract the entry with minimum distance from Priority queue
void extractMin(float** array, int arrayLen) {

  //    if(arrayLen<1)
  //        printf("Error: heap underflow\n");
  array[0] = array[arrayLen - 1];
  heapify(array, arrayLen - 1, 0);
}

/*
 * num_iteration : the total number of iterations for maze route to run
 * round : the number of maze route stages runned
 */

void SPRoute::updateCongestionHistory(int upType) {
  int i, j, grid, maxlimit;
  float overflow;

  maxlimit = 0;

  if(verbose_ > none)
    printf("updateType %d\n", upType);

  if (upType == 1) {
    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {
        grid     = i * (xGrid - 1) + j;
        overflow = h_edges[grid].usage - h_edges[grid].cap;

        if (overflow > 0) {
          h_edges[grid].last_usage += overflow;
          h_edges[grid].congCNT++;
        } else {
          if (!stopDEC) {
            h_edges[grid].last_usage = h_edges[grid].last_usage * 0.9;
          }
        }
        maxlimit = max(maxlimit, h_edges[grid].last_usage);
      }
    }

    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {
        grid     = i * xGrid + j;
        overflow = v_edges[grid].usage - v_edges[grid].cap;

        if (overflow > 0) {
          v_edges[grid].last_usage += overflow;
          v_edges[grid].congCNT++;
        } else {
          if (!stopDEC) {
            v_edges[grid].last_usage = v_edges[grid].last_usage * 0.9;
          }
        }
        maxlimit = max(maxlimit, v_edges[grid].last_usage);
      }
    }
  } else if (upType == 2) {
    if (max_adj < ahTH) {
      stopDEC = true;
    } else {
      stopDEC = false;
    }
    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {
        grid     = i * (xGrid - 1) + j;
        overflow = h_edges[grid].usage - h_edges[grid].cap;

        if (overflow > 0) {
          h_edges[grid].congCNT++;
          h_edges[grid].last_usage += overflow;
        } else {
          if (!stopDEC) {
            h_edges[grid].congCNT--;
            h_edges[grid].congCNT    = max(0, h_edges[grid].congCNT);
            h_edges[grid].last_usage = h_edges[grid].last_usage * 0.9;
          }
        }
        maxlimit = max(maxlimit, h_edges[grid].last_usage);
      }
    }

    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {
        grid     = i * xGrid + j;
        overflow = v_edges[grid].usage - v_edges[grid].cap;

        if (overflow > 0) {
          v_edges[grid].congCNT++;
          v_edges[grid].last_usage += overflow;
        } else {
          if (!stopDEC) {
            v_edges[grid].congCNT--;
            v_edges[grid].congCNT    = max(0, v_edges[grid].congCNT);
            v_edges[grid].last_usage = v_edges[grid].last_usage * 0.9;
          }
        }
        maxlimit = max(maxlimit, v_edges[grid].last_usage);
      }
    }

  } else if (upType == 3) {
    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {
        grid     = i * (xGrid - 1) + j;
        overflow = h_edges[grid].usage - h_edges[grid].cap;

        if (overflow > 0) {
          h_edges[grid].congCNT++;
          h_edges[grid].last_usage += overflow;
        } else {
          if (!stopDEC) {
            h_edges[grid].congCNT--;
            h_edges[grid].congCNT = max(0, h_edges[grid].congCNT);
            h_edges[grid].last_usage += overflow;
            h_edges[grid].last_usage = max(h_edges[grid].last_usage, 0);
          }
        }
        maxlimit = max(maxlimit, h_edges[grid].last_usage);
      }
    }

    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {
        grid     = i * xGrid + j;
        overflow = v_edges[grid].usage - v_edges[grid].cap;

        if (overflow > 0) {
          v_edges[grid].congCNT++;
          v_edges[grid].last_usage += overflow;
        } else {
          if (!stopDEC) {
            v_edges[grid].congCNT--;
            v_edges[grid].last_usage += overflow;
            v_edges[grid].last_usage = max(v_edges[grid].last_usage, 0);
          }
        }
        maxlimit = max(maxlimit, v_edges[grid].last_usage);
      }
    }

  } else if (upType == 4) {
    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {
        grid     = i * (xGrid - 1) + j;
        overflow = h_edges[grid].usage - h_edges[grid].cap;

        if (overflow > 0) {
          h_edges[grid].congCNT++;
          h_edges[grid].last_usage += overflow;
        } else {
          if (!stopDEC) {
            h_edges[grid].congCNT--;
            h_edges[grid].congCNT    = max(0, h_edges[grid].congCNT);
            h_edges[grid].last_usage = h_edges[grid].last_usage * 0.9;
          }
        }
        maxlimit = max(maxlimit, h_edges[grid].last_usage);
      }
    }

    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {
        grid     = i * xGrid + j;
        overflow = v_edges[grid].usage - v_edges[grid].cap;

        if (overflow > 0) {
          v_edges[grid].congCNT++;
          v_edges[grid].last_usage += overflow;
        } else {
          if (!stopDEC) {
            v_edges[grid].congCNT--;
            v_edges[grid].congCNT    = max(0, v_edges[grid].congCNT);
            v_edges[grid].last_usage = v_edges[grid].last_usage * 0.9;
          }
        }
        maxlimit = max(maxlimit, v_edges[grid].last_usage);
      }
    }
    //	if (maxlimit < 20) {
    //		stopDEC = true;
    //	}
  }

  max_adj = maxlimit;

  if(verbose_ > none)
    printf("max value %d stop %d\n", maxlimit, stopDEC);
}

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
               float** d1, int** corrEdge, bool** inRegion) {
  int i, j, d, numNodes, n1, n2, x1, y1, x2, y2;
  int nbr, nbrX, nbrY, cur, edge;
  int x_grid, y_grid;
  int queuehead, queuetail, *queue;
  bool* visited;
  TreeEdge* treeedges;
  TreeNode* treenodes;
  Route* route;

  for (i = regionY1; i <= regionY2; i++) {
    for (j = regionX1; j <= regionX2; j++)
      inRegion[i][j] = true;
  }

  treeedges = sttrees[netID].edges;
  treenodes = sttrees[netID].nodes;
  d         = sttrees[netID].deg;

  n1 = treeedges[edgeID].n1;
  n2 = treeedges[edgeID].n2;
  x1 = treenodes[n1].x;
  y1 = treenodes[n1].y;
  x2 = treenodes[n2].x;
  y2 = treenodes[n2].y;

  // if(netID == 14628)
  //    printf("net: %d edge: %d src: %d %d dst: %d %d d: %d\n", netID, edgeID,
  //    y1, x1, y2, x2, d);
  pq1.clear();
  v2.clear(); // Michael
  if (d == 2) // 2-pin net
  {
    d1[y1][x1] = 0;
    pq1.push({&(d1[y1][x1]), 0});
    v2.push_back(y2 * xGrid + x2);
  } else // net with more than 2 pins
  {
    numNodes = 2 * d - 2;

    visited = (bool*)calloc(numNodes, sizeof(bool));
    for (i = 0; i < numNodes; i++)
      visited[i] = false;

    queue = (int*)calloc(numNodes, sizeof(int));

    // find all the grids on tree edges in subtree t1 (connecting to n1) and put
    // them into heap1
    if (n1 < d) // n1 is a Pin node
    {
      // just need to put n1 itself into heap1
      d1[y1][x1] = 0;
      pq1.push({&(d1[y1][x1]), 0});
      visited[n1] = true;
    } else // n1 is a Steiner node
    {
      queuehead = queuetail = 0;

      // add n1 into heap1
      d1[y1][x1] = 0;
      // if(netID == 252163 && edgeID == 51)
      //    printf("y: %d x: %d\n", y1, x1);
      pq1.push({&(d1[y1][x1]), 0});
      visited[n1] = true;

      // add n1 into the queue
      queue[queuetail] = n1;
      queuetail++;

      // loop to find all the edges in subtree t1
      while (queuetail > queuehead) {
        // get cur node from the queuehead
        cur = queue[queuehead];
        queuehead++;
        visited[cur] = true;
        if (cur >= d) // cur node is a Steiner node
        {
          for (i = 0; i < 3; i++) {
            nbr  = treenodes[cur].nbr[i];
            edge = treenodes[cur].edge[i];
            if (nbr != n2) // not n2
            {
              if (visited[nbr] == false) {
                // put all the grids on the two adjacent tree edges into heap1
                if (treeedges[edge].route.routelen > 0) // not a degraded edge
                {
                  // put nbr into heap1 if in enlarged region
                  if (inRegion[treenodes[nbr].y][treenodes[nbr].x]) {
                    nbrX           = treenodes[nbr].x;
                    nbrY           = treenodes[nbr].y;
                    d1[nbrY][nbrX] = 0;
                    // if(netID == 252163 && edgeID == 51)
                    //    printf("y: %d x: %d\n", nbrY, nbrX);
                    pq1.push({&(d1[nbrY][nbrX]), 0});
                    corrEdge[nbrY][nbrX] = edge;
                  }

                  // the coordinates of two end nodes of the edge

                  route = &(treeedges[edge].route);
                  if (route->type == MAZEROUTE) {
                    for (j = 1; j < route->routelen;
                         j++) // don't put edge_n1 and edge_n2 into heap1
                    {
                      x_grid = route->gridsX[j];
                      y_grid = route->gridsY[j];

                      if (inRegion[y_grid][x_grid]) {
                        d1[y_grid][x_grid] = 0;
                        // if(netID == 252163 && edgeID == 51)
                        //    printf("y: %d x: %d\n", y_grid, x_grid);
                        pq1.push({&(d1[y_grid][x_grid]), 0});
                        corrEdge[y_grid][x_grid] = edge;
                      }
                    }
                  } // if MAZEROUTE
                  else {
                    printf("Setup Heap: not maze routing\n");
                  }
                } // if not a degraded edge (len>0)

                // add the neighbor of cur node into queue
                queue[queuetail] = nbr;
                queuetail++;
              } // if the node is not visited
            }   // if nbr!=n2
          }     // loop i (3 neigbors for cur node)
        }       // if cur node is a Steiner nodes
      }         // while queue is not empty
    }           // else n1 is not a Pin node

    // find all the grids on subtree t2 (connect to n2) and put them into heap2
    // find all the grids on tree edges in subtree t2 (connecting to n2) and put
    // them into heap2
    if (n2 < d) // n2 is a Pin node
    {
      // just need to put n2 itself into heap2
      v2.push_back(y2 * xGrid + x2);
      // if(netID == 14628)
      //    printf("y: %d x: %d \n", y2, x2);
      visited[n2] = true;
    } else // n2 is a Steiner node
    {
      queuehead = queuetail = 0;

      // add n2 into heap2
      v2.push_back(y2 * xGrid + x2);
      // if(netID == 252163 && edgeID == 51)
      //    printf("dst y: %d x: %d \n", y2, x2);
      visited[n2] = true;

      // add n2 into the queue
      queue[queuetail] = n2;
      queuetail++;

      // loop to find all the edges in subtree t2
      while (queuetail > queuehead) {
        // get cur node form queuehead
        cur          = queue[queuehead];
        visited[cur] = true;
        queuehead++;

        if (cur >= d) // cur node is a Steiner node
        {
          for (i = 0; i < 3; i++) {
            nbr  = treenodes[cur].nbr[i];
            edge = treenodes[cur].edge[i];
            if (nbr != n1) // not n1
            {
              if (visited[nbr] == false) {
                // put all the grids on the two adjacent tree edges into heap2
                if (treeedges[edge].route.routelen > 0) // not a degraded edge
                {
                  // put nbr into heap2
                  if (inRegion[treenodes[nbr].y][treenodes[nbr].x]) {
                    nbrX = treenodes[nbr].x;
                    nbrY = treenodes[nbr].y;
                    v2.push_back(nbrY * xGrid + nbrX);
                    // if(netID == 252163 && edgeID == 51)
                    //    printf("dst y: %d x: %d\n", nbrY, nbrX);
                    corrEdge[nbrY][nbrX] = edge;
                  }

                  // the coordinates of two end nodes of the edge

                  route = &(treeedges[edge].route);
                  if (route->type == MAZEROUTE) {
                    for (j = 1; j < route->routelen;
                         j++) // don't put edge_n1 and edge_n2 into heap2
                    {
                      x_grid = route->gridsX[j];
                      y_grid = route->gridsY[j];
                      if (inRegion[y_grid][x_grid]) {
                        v2.push_back(y_grid * xGrid + x_grid);
                        // if(netID == 252163 && edgeID == 51)
                        //    printf("dst y: %d x: %d\n", y_grid, x_grid);
                        corrEdge[y_grid][x_grid] = edge;
                      }
                    }
                  } // if MAZEROUTE
                  else {
                    printf("Setup Heap: not maze routing\n");
                  }
                } // if the edge is not degraded (len>0)

                // add the neighbor of cur node into queue
                queue[queuetail] = nbr;
                queuetail++;
              } // if the node is not visited
            }   // if nbr!=n1
          }     // loop i (3 neigbors for cur node)
        }       // if cur node is a Steiner nodes
      }         // while queue is not empty
    }           // else n2 is not a Pin node

    free(queue);
    free(visited);
  } // net with more than two pins

  for (i = regionY1; i <= regionY2; i++) {
    for (j = regionX1; j <= regionX2; j++)
      inRegion[i][j] = false;
  }
}

int copyGrids(TreeNode* treenodes, int n1, TreeEdge* treeedges, int edge_n1n2,
              int* gridsX_n1n2, int* gridsY_n1n2) {
  int i, cnt;
  int n1x, n1y;

  n1x = treenodes[n1].x;
  n1y = treenodes[n1].y;

  cnt = 0;
  if (treeedges[edge_n1n2].n1 == n1) // n1 is the first node of (n1, n2)
  {
    if (treeedges[edge_n1n2].route.type == MAZEROUTE) {
      for (i = 0; i <= treeedges[edge_n1n2].route.routelen; i++) {
        gridsX_n1n2[cnt] = treeedges[edge_n1n2].route.gridsX[i];
        gridsY_n1n2[cnt] = treeedges[edge_n1n2].route.gridsY[i];
        cnt++;
      }
    }    // MAZEROUTE
    else // NOROUTE
    {
      gridsX_n1n2[cnt] = n1x;
      gridsY_n1n2[cnt] = n1y;
      cnt++;
    }
  }    // if n1 is the first node of (n1, n2)
  else // n2 is the first node of (n1, n2)
  {
    if (treeedges[edge_n1n2].route.type == MAZEROUTE) {
      for (i = treeedges[edge_n1n2].route.routelen; i >= 0; i--) {
        gridsX_n1n2[cnt] = treeedges[edge_n1n2].route.gridsX[i];
        gridsY_n1n2[cnt] = treeedges[edge_n1n2].route.gridsY[i];
        cnt++;
      }
    }    // MAZEROUTE
    else // NOROUTE
    {
      gridsX_n1n2[cnt] = n1x;
      gridsY_n1n2[cnt] = n1y;
      cnt++;
    } // MAZEROUTE
  }

  return (cnt);
}

void updateRouteType1(TreeNode* treenodes, int n1, int A1, int A2, int E1x,
                      int E1y, TreeEdge* treeedges, int edge_n1A1,
                      int edge_n1A2) {
  using namespace galois::substrate;
  int i, cnt, A1x, A1y, A2x, A2y;
  int cnt_n1A1, cnt_n1A2, E1_pos = 0;
  // int gridsXY[4 * (xGrid + yGrid)];

  int gridsX_n1A1[2 * (xGrid + yGrid)], gridsY_n1A1[2 * (xGrid + yGrid)],
      gridsX_n1A2[2 * (xGrid + yGrid)], gridsY_n1A2[2 * (xGrid + yGrid)];

  /*int* gridsX_n1A1 = gridsXY;
  int* gridsY_n1A1 = gridsXY + xGrid + yGrid;
  int* gridsX_n1A2 = gridsXY + 2 * (xGrid + yGrid);
  int* gridsY_n1A2 = gridsXY + 3 * (xGrid + yGrid);*/

  /*LAptr gridsX_n1A1_LA, gridsY_n1A1_LA, gridsX_n1A2_LA, gridsY_n1A2_LA;
  gridsX_n1A1_LA = largeMallocLocal((xGrid + yGrid) * sizeof(int));
  gridsY_n1A1_LA = largeMallocLocal((xGrid + yGrid) * sizeof(int));
  gridsX_n1A2_LA = largeMallocLocal((xGrid + yGrid) * sizeof(int));
  gridsY_n1A2_LA = largeMallocLocal((xGrid + yGrid) * sizeof(int));

  int* gridsX_n1A1 = reinterpret_cast<int*> (gridsX_n1A1_LA.get());
  int* gridsY_n1A1 = reinterpret_cast<int*> (gridsY_n1A1_LA.get());
  int* gridsX_n1A2 = reinterpret_cast<int*> (gridsX_n1A2_LA.get());
  int* gridsY_n1A2 = reinterpret_cast<int*> (gridsY_n1A2_LA.get());*/

  A1x = treenodes[A1].x;
  A1y = treenodes[A1].y;
  A2x = treenodes[A2].x;
  A2y = treenodes[A2].y;

  // copy all the grids on (n1, A1) and (n2, A2) to tmp arrays, and keep the
  // grids order A1->n1->A2 copy (n1, A1)
  cnt_n1A1 =
      copyGrids(treenodes, A1, treeedges, edge_n1A1, gridsX_n1A1, gridsY_n1A1);

  // copy (n1, A2)
  cnt_n1A2 =
      copyGrids(treenodes, n1, treeedges, edge_n1A2, gridsX_n1A2, gridsY_n1A2);

  // update route for (n1, A1) and (n1, A2)
  // find the index of E1 in (n1, A1)
  for (i = 0; i < cnt_n1A1; i++) {
    if (gridsX_n1A1[i] == E1x && gridsY_n1A1[i] == E1y) // reach the E1
    {
      E1_pos = i;
      break;
    }
  }

  // reallocate memory for route.gridsX and route.gridsY
  if (treeedges[edge_n1A1].route.type ==
      MAZEROUTE) // if originally allocated, free them first
  {
    free(treeedges[edge_n1A1].route.gridsX);
    free(treeedges[edge_n1A1].route.gridsY);
  }
  treeedges[edge_n1A1].route.gridsX =
      (short*)calloc((E1_pos + 1), sizeof(short));
  treeedges[edge_n1A1].route.gridsY =
      (short*)calloc((E1_pos + 1), sizeof(short));

  if (A1x <= E1x) {
    cnt = 0;
    for (i = 0; i <= E1_pos; i++) {
      treeedges[edge_n1A1].route.gridsX[cnt] = gridsX_n1A1[i];
      treeedges[edge_n1A1].route.gridsY[cnt] = gridsY_n1A1[i];
      cnt++;
    }
    treeedges[edge_n1A1].n1 = A1;
    treeedges[edge_n1A1].n2 = n1;
  } else {
    cnt = 0;
    for (i = E1_pos; i >= 0; i--) {
      treeedges[edge_n1A1].route.gridsX[cnt] = gridsX_n1A1[i];
      treeedges[edge_n1A1].route.gridsY[cnt] = gridsY_n1A1[i];
      cnt++;
    }
    treeedges[edge_n1A1].n1 = n1;
    treeedges[edge_n1A1].n2 = A1;
  }

  treeedges[edge_n1A1].route.type     = MAZEROUTE;
  treeedges[edge_n1A1].route.routelen = E1_pos;
  treeedges[edge_n1A1].len            = ADIFF(A1x, E1x) + ADIFF(A1y, E1y);

  // reallocate memory for route.gridsX and route.gridsY
  if (treeedges[edge_n1A2].route.type ==
      MAZEROUTE) // if originally allocated, free them first
  {
    free(treeedges[edge_n1A2].route.gridsX);
    free(treeedges[edge_n1A2].route.gridsY);
  }
  treeedges[edge_n1A2].route.gridsX =
      (short*)calloc((cnt_n1A1 + cnt_n1A2 - E1_pos - 1), sizeof(short));
  treeedges[edge_n1A2].route.gridsY =
      (short*)calloc((cnt_n1A1 + cnt_n1A2 - E1_pos - 1), sizeof(short));

  if (E1x <= A2x) {
    cnt = 0;
    for (i = E1_pos; i < cnt_n1A1; i++) {
      treeedges[edge_n1A2].route.gridsX[cnt] = gridsX_n1A1[i];
      treeedges[edge_n1A2].route.gridsY[cnt] = gridsY_n1A1[i];
      cnt++;
    }
    for (i = 1; i < cnt_n1A2; i++) // 0 is n1 again, so no repeat
    {
      treeedges[edge_n1A2].route.gridsX[cnt] = gridsX_n1A2[i];
      treeedges[edge_n1A2].route.gridsY[cnt] = gridsY_n1A2[i];
      cnt++;
    }
    treeedges[edge_n1A2].n1 = n1;
    treeedges[edge_n1A2].n2 = A2;
  } else {
    cnt = 0;
    for (i = cnt_n1A2 - 1; i >= 1; i--) // 0 is n1 again, so no repeat
    {
      treeedges[edge_n1A2].route.gridsX[cnt] = gridsX_n1A2[i];
      treeedges[edge_n1A2].route.gridsY[cnt] = gridsY_n1A2[i];
      cnt++;
    }
    for (i = cnt_n1A1 - 1; i >= E1_pos; i--) {
      treeedges[edge_n1A2].route.gridsX[cnt] = gridsX_n1A1[i];
      treeedges[edge_n1A2].route.gridsY[cnt] = gridsY_n1A1[i];
      cnt++;
    }
    treeedges[edge_n1A2].n1 = A2;
    treeedges[edge_n1A2].n2 = n1;
  }
  treeedges[edge_n1A2].route.type     = MAZEROUTE;
  treeedges[edge_n1A2].route.routelen = cnt - 1;
  treeedges[edge_n1A2].len            = ADIFF(A2x, E1x) + ADIFF(A2y, E1y);
}

void updateRouteType2(TreeNode* treenodes, int n1, int A1, int A2, int C1,
                      int C2, int E1x, int E1y, TreeEdge* treeedges,
                      int edge_n1A1, int edge_n1A2, int edge_C1C2) {
  int i, cnt, A1x, A1y, A2x, A2y, C1x, C1y, C2x, C2y;
  int edge_n1C1, edge_n1C2, edge_A1A2;
  int cnt_n1A1, cnt_n1A2, cnt_C1C2, E1_pos = 0;
  int len_A1A2, len_n1C1, len_n1C2;
  int gridsX_n1A1[2 * (xGrid + yGrid)], gridsY_n1A1[2 * (xGrid + yGrid)];
  int gridsX_n1A2[2 * (xGrid + yGrid)], gridsY_n1A2[2 * (xGrid + yGrid)];
  int gridsX_C1C2[2 * (xGrid + yGrid)], gridsY_C1C2[2 * (xGrid + yGrid)];

  A1x = treenodes[A1].x;
  A1y = treenodes[A1].y;
  A2x = treenodes[A2].x;
  A2y = treenodes[A2].y;
  C1x = treenodes[C1].x;
  C1y = treenodes[C1].y;
  C2x = treenodes[C2].x;
  C2y = treenodes[C2].y;

  edge_n1C1 = edge_n1A1;
  edge_n1C2 = edge_n1A2;
  edge_A1A2 = edge_C1C2;

  // combine (n1, A1) and (n1, A2) into (A1, A2), A1 is the first node and A2 is
  // the second grids order A1->n1->A2 copy (A1, n1)
  cnt_n1A1 =
      copyGrids(treenodes, A1, treeedges, edge_n1A1, gridsX_n1A1, gridsY_n1A1);

  // copy (n1, A2)
  cnt_n1A2 =
      copyGrids(treenodes, n1, treeedges, edge_n1A2, gridsX_n1A2, gridsY_n1A2);

  // copy all the grids on (C1, C2) to gridsX_C1C2[] and gridsY_C1C2[]
  cnt_C1C2 =
      copyGrids(treenodes, C1, treeedges, edge_C1C2, gridsX_C1C2, gridsY_C1C2);

  // combine grids on original (A1, n1) and (n1, A2) to new (A1, A2)
  // allocate memory for gridsX[] and gridsY[] of edge_A1A2
  if (treeedges[edge_A1A2].route.type == MAZEROUTE) {
    free(treeedges[edge_A1A2].route.gridsX);
    free(treeedges[edge_A1A2].route.gridsY);
  }
  len_A1A2 = cnt_n1A1 + cnt_n1A2 - 1;

  treeedges[edge_A1A2].route.gridsX   = (short*)calloc(len_A1A2, sizeof(short));
  treeedges[edge_A1A2].route.gridsY   = (short*)calloc(len_A1A2, sizeof(short));
  treeedges[edge_A1A2].route.routelen = len_A1A2 - 1;
  treeedges[edge_A1A2].len            = ADIFF(A1x, A2x) + ADIFF(A1y, A2y);

  cnt = 0;
  for (i = 0; i < cnt_n1A1; i++) {
    treeedges[edge_A1A2].route.gridsX[cnt] = gridsX_n1A1[i];
    treeedges[edge_A1A2].route.gridsY[cnt] = gridsY_n1A1[i];
    cnt++;
  }
  for (i = 1; i < cnt_n1A2; i++) // do not repeat point n1
  {
    treeedges[edge_A1A2].route.gridsX[cnt] = gridsX_n1A2[i];
    treeedges[edge_A1A2].route.gridsY[cnt] = gridsY_n1A2[i];
    cnt++;
  }

  // find the index of E1 in (C1, C2)
  for (i = 0; i < cnt_C1C2; i++) {
    if (gridsX_C1C2[i] == E1x && gridsY_C1C2[i] == E1y) {
      E1_pos = i;
      break;
    }
  }

  // allocate memory for gridsX[] and gridsY[] of edge_n1C1 and edge_n1C2
  if (treeedges[edge_n1C1].route.type == MAZEROUTE) {
    free(treeedges[edge_n1C1].route.gridsX);
    free(treeedges[edge_n1C1].route.gridsY);
  }
  len_n1C1                            = E1_pos + 1;
  treeedges[edge_n1C1].route.gridsX   = (short*)calloc(len_n1C1, sizeof(short));
  treeedges[edge_n1C1].route.gridsY   = (short*)calloc(len_n1C1, sizeof(short));
  treeedges[edge_n1C1].route.routelen = len_n1C1 - 1;
  treeedges[edge_n1C1].len            = ADIFF(C1x, E1x) + ADIFF(C1y, E1y);

  if (treeedges[edge_n1C2].route.type == MAZEROUTE) {
    free(treeedges[edge_n1C2].route.gridsX);
    free(treeedges[edge_n1C2].route.gridsY);
  }
  len_n1C2                            = cnt_C1C2 - E1_pos;
  treeedges[edge_n1C2].route.gridsX   = (short*)calloc(len_n1C2, sizeof(short));
  treeedges[edge_n1C2].route.gridsY   = (short*)calloc(len_n1C2, sizeof(short));
  treeedges[edge_n1C2].route.routelen = len_n1C2 - 1;
  treeedges[edge_n1C2].len            = ADIFF(C2x, E1x) + ADIFF(C2y, E1y);

  // split original (C1, C2) to (C1, n1) and (n1, C2)
  cnt = 0;
  for (i = 0; i <= E1_pos; i++) {
    treeedges[edge_n1C1].route.gridsX[i] = gridsX_C1C2[i];
    treeedges[edge_n1C1].route.gridsY[i] = gridsY_C1C2[i];
    cnt++;
  }

  cnt = 0;
  for (i = E1_pos; i < cnt_C1C2; i++) {
    treeedges[edge_n1C2].route.gridsX[cnt] = gridsX_C1C2[i];
    treeedges[edge_n1C2].route.gridsY[cnt] = gridsY_C1C2[i];
    cnt++;
  }
}

void reInitTree(int netID) {
  int deg, numEdges, edgeID, d, j;
  TreeEdge* treeedge;
  Tree rsmt;
  int x[MAXNETDEG], y[MAXNETDEG];

  // printf("re init tree for net %d\n",netID);

  newRipupNet(netID);

  deg      = sttrees[netID].deg;
  numEdges = 2 * deg - 3;
  for (edgeID = 0; edgeID < numEdges; edgeID++) {
    treeedge = &(sttrees[netID].edges[edgeID]);
    if (treeedge->len > 0) {
      free(treeedge->route.gridsX);
      free(treeedge->route.gridsY);
      free(treeedge->route.gridsL);
    }
  }
  free(sttrees[netID].nodes);
  free(sttrees[netID].edges);

  // printf("old tree component freed\n");

  d = nets[netID]->deg;
  // printf("net deg %d\n",d);
  // fflush(stdout);
  for (j = 0; j < d; j++) {
    x[j] = nets[netID]->pinX[j];
    y[j] = nets[netID]->pinY[j];
  }
  // printf("before flute\n");
  // fflush(stdout);
  fluteCongest(netID, d, x, y, 2, 1.2, &rsmt);
  // printf("fluted worked\n");
  // fflush(stdout);
  if (d > 3) {
    edgeShiftNew(&rsmt);
    // printf("edge shifted\n");
  }
  // fflush(stdout);
  copyStTree(netID, rsmt);
  // printf("tree copied\n");
  // fflush(stdout);
  newrouteLInMaze(netID);

  // fflush(stdout);
  convertToMazerouteNet(netID);
  // printf("L to mzed converted\n");
  // fflush(stdout);
  // checkRoute2DTree(netID);
  // printf("tree double checked\n");
  // fflush(stdout);
}




int SPRoute::getOverflow2Dmaze(int* maxOverflow, int* tUsage, bool after_maze) {
  int i, j, grid, overflow, max_overflow, H_overflow, max_H_overflow,
      V_overflow, max_V_overflow, numedges = 0;
  int total_usage, total_cap;
  bool debug = false;
  // get overflow
  overflow = max_overflow = H_overflow = max_H_overflow = V_overflow =
      max_V_overflow                                    = 0;

  total_usage = 0;
  total_cap   = 0;


  for (i = 0; i < yGrid; i++) {
    for (j = 0; j < xGrid - 1; j++) {
      grid = i * (xGrid - 1) + j;
      total_usage += h_edges[grid].usage;
      overflow = h_edges[grid].usage - h_edges[grid].cap;
      total_cap += h_edges[grid].cap;
      if (overflow > 0) {
        H_overflow += overflow;
        max_H_overflow = max(max_H_overflow, overflow);
        numedges++;
        if(debug && h_edges[grid].cap == 0 && after_maze) {
          cout << "ERROR: using OBS H!" << j << " " << i << endl;
          exit(1);
        }
        
      }
    }
  }
  //    fprintf(fpv, "\nVertical Congestion\n");
  for (i = 0; i < yGrid - 1; i++) {
    for (j = 0; j < xGrid; j++) {
      grid = i * xGrid + j;
      total_usage += v_edges[grid].usage;
      overflow = v_edges[grid].usage - v_edges[grid].cap;
      total_cap += v_edges[grid].cap;
      if (overflow > 0) {
        V_overflow += overflow;
        max_V_overflow = max(max_V_overflow, overflow);
        numedges++;
        if(debug && v_edges[grid].cap == 0 && after_maze) {
          cout << "ERROR: using OBS V!" << j << " " << i << endl;
          exit(1);
        }
      }
    }
  }
  

  max_overflow  = max(max_H_overflow, max_V_overflow);
  totalOverflow = H_overflow + V_overflow;
  *maxOverflow  = max_overflow; 

  if(verbose_ > none) {
    printf("total Usage   : %d\n", (int)total_usage);
    printf("Max H Overflow: %d\n", max_H_overflow);
    printf("Max V Overflow: %d\n", max_V_overflow);
    printf("Max Overflow  : %d\n", max_overflow);
    printf("Num Overflow e: %d\n", numedges);
    printf("H   Overflow  : %d\n", H_overflow);
    printf("V   Overflow  : %d\n", V_overflow);
    printf("Final Overflow: %d\n\n", totalOverflow);
  }

  *tUsage = total_usage;

  if (total_usage > 800000) {
    ahTH = 30;
  } else {
    ahTH = 20;
  }

  return (totalOverflow);
}

void checkUsageCorrectness() {
  int* vedge_usage = new int[xGrid * (yGrid - 1)];
  int* hedge_usage = new int[(xGrid - 1) * yGrid];
  memset(vedge_usage, 0, xGrid * (yGrid - 1) * sizeof(int));
  memset(hedge_usage, 0, (xGrid - 1) * yGrid * sizeof(int));

  for (int netID = 0; netID < numValidNets; netID++) {
    // maze routing for multi-source, multi-destination

    int deg, edgeID, n1, n2, n1x, n1y, n2x, n2y, num_edges;

    TreeEdge *treeedges, *treeedge;
    TreeNode* treenodes;

    deg = sttrees[netID].deg;

    treeedges = sttrees[netID].edges;
    treenodes = sttrees[netID].nodes;
    // loop for all the tree edges (2*deg-3)
    num_edges = 2 * deg - 3;

    for (edgeID = 0; edgeID < num_edges; edgeID++) {
      treeedge = &(treeedges[edgeID]);

      n1            = treeedge->n1;
      n2            = treeedge->n2;
      n1x           = treenodes[n1].x;
      n1y           = treenodes[n1].y;
      n2x           = treenodes[n2].x;
      n2y           = treenodes[n2].y;
      treeedge->len = ADIFF(n2x, n1x) + ADIFF(n2y, n1y);

      if (treeedge->len > 0) // only route the non-degraded edges (len>0)
      {
        if (treeedge->route.type == MAZEROUTE) {
          short* gridsX = treeedge->route.gridsX;
          short* gridsY = treeedge->route.gridsY;

          // std::cout << "net: " << netID << " route lenth: " <<
          // treeedge->route.routelen << std::endl;

          for (int i = 0; i < treeedge->route.routelen; i++) {
            // std::cout << "path: " << gridsX[i] << "," << gridsY[i] << "<->"
            // << gridsX[i + 1] << "," << gridsY[i + 1] << std::endl;
            if (gridsX[i] == gridsX[i + 1]) {
              int ymin = min(gridsY[i], gridsY[i + 1]);
              vedge_usage[ymin * xGrid + gridsX[i]]++;
            } else if (gridsY[i] == gridsY[i + 1]) {
              int xmin = min(gridsX[i], gridsX[i + 1]);
              hedge_usage[gridsY[i] * (xGrid - 1) + xmin]++;
            } else {
              std::cout << "usage correctness: net not connected!" << std::endl;
            }
          }
        } else {
          std::cout << "usage correctness: not maze route!" << std::endl;
          exit(1);
        }
      }
    }
  }
  int same = 0;
  int diff = 0;
  for (int i = 0; i < yGrid; i++) {
    for (int j = 0; j < xGrid - 1; j++) {
      int grid = i * (xGrid - 1) + j;
      if (hedge_usage[grid] == h_edges[grid].usage) {
        same++;
      } else {
        diff++;
        // std::cout << "h edge diff: " << j << ", " << i << " check: " <<
        // hedge_usage[grid] << " actual: " << h_edges[grid].usage << std::endl;
      }
    }
  }

  for (int i = 0; i < yGrid - 1; i++) {
    for (int j = 0; j < xGrid; j++) {
      int grid = i * xGrid + j;
      if (vedge_usage[grid] == v_edges[grid].usage) {
        same++;
      } else {
        diff++;
        // std::cout << "v edge diff: " << j << ", " << i << " check: " <<
        // vedge_usage[grid] << " actual: " << v_edges[grid].usage << std::endl;
      }
    }
  }

  std::cout << "same: " << same << " diff: " << diff << std::endl;

  delete[] vedge_usage;
  delete[] hedge_usage;
}

int SPRoute::getOverflow2D(int* maxOverflow) {
  int i, j, grid, overflow, max_overflow, H_overflow, max_H_overflow,
      V_overflow, max_V_overflow, numedges;
  int total_usage, total_cap, hCap, vCap;

  // get overflow
  overflow = max_overflow = H_overflow = max_H_overflow = V_overflow =
      max_V_overflow                                    = 0;
  hCap = vCap = numedges = 0;

  total_usage = 0;
  total_cap   = 0;
  //    fprintf(fph, "Horizontal Congestion\n");
  for (i = 0; i < yGrid; i++) {
    for (j = 0; j < xGrid - 1; j++) {
      grid = i * (xGrid - 1) + j;
      total_usage += h_edges[grid].est_usage;
      overflow = h_edges[grid].est_usage - h_edges[grid].cap;
      total_cap += h_edges[grid].cap;
      hCap += h_edges[grid].cap;
      if (overflow > 0) {
        H_overflow += overflow;
        max_H_overflow = max(max_H_overflow, overflow);
        numedges++;
      }
    }
  }
  //    fprintf(fpv, "\nVertical Congestion\n");
  for (i = 0; i < yGrid - 1; i++) {
    for (j = 0; j < xGrid; j++) {
      grid = i * xGrid + j;
      total_usage += v_edges[grid].est_usage;
      overflow = v_edges[grid].est_usage - v_edges[grid].cap;
      total_cap += v_edges[grid].cap;
      vCap += v_edges[grid].cap;
      if (overflow > 0) {
        V_overflow += overflow;
        max_V_overflow = max(max_V_overflow, overflow);
        numedges++;
      }
    }
  }

  max_overflow  = max(max_H_overflow, max_V_overflow);
  totalOverflow = H_overflow + V_overflow;
  *maxOverflow  = max_overflow;

  if (total_usage > 800000) {
    ahTH = 30;
  } else {
    ahTH = 20;
  }

  if(verbose_ > none) {
    printf("total hCap    : %d\n", hCap);
    printf("total vCap    : %d\n", vCap);
    printf("total Usage   : %d\n", (int)total_usage);
    printf("Max H Overflow: %d\n", max_H_overflow);
    printf("Max V Overflow: %d\n", max_V_overflow);
    printf("Max Overflow  : %d\n", max_overflow);
    printf("Num Overflow e: %d\n", numedges);
    printf("H   Overflow  : %d\n", H_overflow);
    printf("V   Overflow  : %d\n", V_overflow);
    printf("Final Overflow: %d\n\n", totalOverflow);
  }

  return (totalOverflow);
}

int SPRoute::getOverflow3D(void) {
  int i, j, k, grid, overflow, max_overflow, H_overflow, max_H_overflow,
      V_overflow, max_V_overflow;
  int cap;
  int total_usage;

  // get overflow
  overflow = max_overflow = H_overflow = max_H_overflow = V_overflow =
      max_V_overflow                                    = 0;

  total_usage = 0;
  cap         = 0;
  //    fprintf(fph, "Horizontal Congestion\n");

  for (k = 0; k < numLayers; k++) {
    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {
        grid = i * (xGrid - 1) + j + k * (xGrid - 1) * yGrid;
        total_usage += h_edges3D[grid].usage;
        overflow = h_edges3D[grid].usage - h_edges3D[grid].cap;
        cap += h_edges3D[grid].cap;

        if (overflow > 0) {
          H_overflow += overflow;
          max_H_overflow = max(max_H_overflow, overflow);
        }
      }
    }
    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {
        grid = i * xGrid + j + k * xGrid * (yGrid - 1);
        total_usage += v_edges3D[grid].usage;
        overflow = v_edges3D[grid].usage - v_edges3D[grid].cap;
        cap += v_edges3D[grid].cap;

        if (overflow > 0) {
          V_overflow += overflow;
          max_V_overflow = max(max_V_overflow, overflow);
        }
      }
    }
  }

  max_overflow  = max(max_H_overflow, max_V_overflow);
  totalOverflow = H_overflow + V_overflow;

  if(verbose_ > none) {
    printf("total Usage   : %d\n", total_usage);
    printf("Total Capacity: %d\n", cap);
    printf("Max H Overflow: %d\n", max_H_overflow);
    printf("Max V Overflow: %d\n", max_V_overflow);
    printf("Max Overflow  : %d\n", max_overflow);
    printf("H   Overflow  : %d\n", H_overflow);
    printf("V   Overflow  : %d\n", V_overflow);
    printf("Final Overflow: %d\n\n", totalOverflow);
  }

  return (total_usage);
}


/*void initialCongestionHistory(int round)
{
    int i, j, grid;

    for(i=0; i<yGrid; i++)
    {
        for(j=0; j<xGrid-1; j++)
        {
            grid = i*(xGrid-1)+j;
            h_edges[grid].est_usage -=
((float)h_edges[grid].usage/h_edges[grid].cap);

        }
    }

    for(i=0; i<yGrid-1; i++)
    {
        for(j=0; j<xGrid; j++)
        {
            grid = i*xGrid+j;
            v_edges[grid].est_usage -=
((float)v_edges[grid].usage/v_edges[grid].cap);

        }
    }

}

void reduceCongestionHistory(int round)
{
    int i, j, grid;

    for(i=0; i<yGrid; i++)
    {
        for(j=0; j<xGrid-1; j++)
        {
            grid = i*(xGrid-1)+j;
            h_edges[grid].est_usage -=
0.2*((float)h_edges[grid].usage/h_edges[grid].cap);
        }
    }

    for(i=0; i<yGrid-1; i++)
    {
        for(j=0; j<xGrid; j++)
        {
            grid = i*xGrid+j;
            v_edges[grid].est_usage -=
0.2*((float)v_edges[grid].usage/v_edges[grid].cap);
        }
    }

}*/

void InitEstUsage() {
  int i, j, grid;
  for (i = 0; i < yGrid; i++) {
    for (j = 0; j < xGrid - 1; j++) {
      grid                    = i * (xGrid - 1) + j;
      h_edges[grid].est_usage = 0;
    }
  }
  //    fprintf(fpv, "\nVertical Congestion\n");
  for (i = 0; i < yGrid - 1; i++) {
    for (j = 0; j < xGrid; j++) {
      grid                    = i * xGrid + j;
      v_edges[grid].est_usage = 0;
    }
  }
}

void str_accu(int rnd) {
  int i, j, grid, overflow;
  for (i = 0; i < yGrid; i++) {
    for (j = 0; j < xGrid - 1; j++) {
      grid     = i * (xGrid - 1) + j;
      overflow = h_edges[grid].usage - h_edges[grid].cap;
      if (overflow > 0 || h_edges[grid].congCNT > rnd) {
        h_edges[grid].last_usage += h_edges[grid].congCNT * overflow / 2;
      }
    }
  }
  //    fprintf(fpv, "\nVertical Congestion\n");
  for (i = 0; i < yGrid - 1; i++) {
    for (j = 0; j < xGrid; j++) {
      grid     = i * xGrid + j;
      overflow = v_edges[grid].usage - v_edges[grid].cap;
      if (overflow > 0 || v_edges[grid].congCNT > rnd) {
        v_edges[grid].last_usage += v_edges[grid].congCNT * overflow / 2;
      }
    }
  }
}

void InitLastUsage(int upType) {
  int i, j, grid;
  for (i = 0; i < yGrid; i++) {
    for (j = 0; j < xGrid - 1; j++) {
      grid                     = i * (xGrid - 1) + j;
      h_edges[grid].last_usage = 0;
    }
  }
  //    fprintf(fpv, "\nVertical Congestion\n");
  for (i = 0; i < yGrid - 1; i++) {
    for (j = 0; j < xGrid; j++) {
      grid                     = i * xGrid + j;
      v_edges[grid].last_usage = 0;
    }
  }

  if (upType == 1) {
    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {
        grid                  = i * (xGrid - 1) + j;
        h_edges[grid].congCNT = 0;
      }
    }
    //    fprintf(fpv, "\nVertical Congestion\n");
    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {
        grid                  = i * xGrid + j;
        v_edges[grid].congCNT = 0;
      }
    }
  } else if (upType == 2) {

    for (i = 0; i < yGrid; i++) {
      for (j = 0; j < xGrid - 1; j++) {

        grid = i * (xGrid - 1) + j;
        // if (overflow > 0)
        h_edges[grid].last_usage = h_edges[grid].last_usage * 0.2;
      }
    }
    //    fprintf(fpv, "\nVertical Congestion\n");
    for (i = 0; i < yGrid - 1; i++) {
      for (j = 0; j < xGrid; j++) {

        grid = i * xGrid + j;
        //	if (overflow > 0)
        v_edges[grid].last_usage = v_edges[grid].last_usage * 0.2;
      }
    }
  }
}


} //namespace sproute
