#include "setup_heap.h"

namespace sproute {

void setupHeap_astar_swap(int netID, int edgeID, astar_local_pq& pq1, local_vec& v2,
               int regionX1, int regionX2, int regionY1, int regionY2,
               float** d1, int** corrEdge, bool** inRegion, float astar_weight) {
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

  n1 = treeedges[edgeID].n2;
  n2 = treeedges[edgeID].n1;
  //key: here is changed!

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
    float dst_dist = astar_weight * (fabs(x1 - x2) + fabs(y1 - y2));
    pq1.push({&(d1[y1][x1]), 0, dst_dist});
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
      float dst_dist = astar_weight * (fabs(x1 - x2) + fabs(y1 - y2));
      pq1.push({&(d1[y1][x1]), 0, dst_dist});
      visited[n1] = true;
    } else // n1 is a Steiner node
    {
      queuehead = queuetail = 0;

      // add n1 into heap1
      d1[y1][x1] = 0;
      float dst_dist = astar_weight * (fabs(x1 - x2) + fabs(y1 - y2));
      pq1.push({&(d1[y1][x1]), 0, dst_dist});
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
                    dst_dist = astar_weight * (fabs(nbrX - x2) + fabs(nbrY - y2));
                    pq1.push({&(d1[nbrY][nbrX]), 0, dst_dist});
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
                        dst_dist = astar_weight * (fabs(x_grid - x2) + fabs(y_grid - y2));
                        pq1.push({&(d1[y_grid][x_grid]), 0, dst_dist});
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
    if (1) // in astar, only 1 destination node (was n2 < d) 
    {
      // just need to put n2 itself into heap2
      v2.push_back(y2 * xGrid + x2);
      visited[n2] = true;
    } 

    free(queue);
    free(visited);
  } // net with more than two pins

  for (i = regionY1; i <= regionY2; i++) {
    for (j = regionX1; j <= regionX2; j++)
      inRegion[i][j] = false;
  }
}


void setupHeap_steiner(int netID, int edgeID, astar_local_pq& pq1, local_vec& v2,
               int regionX1, int regionX2, int regionY1, int regionY2,
               float** d1, int** corrEdge, bool** inRegion, float astar_weight) {
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
    pq1.push({&(d1[y1][x1]), 0, 0});
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
      pq1.push({&(d1[y1][x1]), 0, 0});
      visited[n1] = true;
    } else // n1 is a Steiner node
    {
      queuehead = queuetail = 0;

      // add n1 into heap1
      d1[y1][x1] = 0;
      // if(netID == 252163 && edgeID == 51)
      //    printf("y: %d x: %d\n", y1, x1);
      pq1.push({&(d1[y1][x1]), 0, 0});
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
                    pq1.push({&(d1[nbrY][nbrX]), 0, 0});
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
                        pq1.push({&(d1[y_grid][x_grid]), 0, 0});
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

} //namespace sproute
