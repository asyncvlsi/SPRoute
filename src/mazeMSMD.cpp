#include "maze.h"
#include "sproute.h"
namespace sproute {

void SPRoute::UndoneFilter(galois::LargeArray<bool> &done) {
    galois::GAccumulator<uint32_t> count;
    galois::do_all(galois::iterate((int)0, numValidNets),[&](int netID) {
          
          int deg = sttrees[netID].deg;

          TreeEdge* treeedges = sttrees[netID].edges;
          TreeNode* treenodes = sttrees[netID].nodes;
          int num_edges = 2 * deg - 3;

			    bool flag = true;
            
          for (int edgeID = 0; edgeID < num_edges; edgeID++) {
              TreeEdge* treeedge = &(treeedges[edgeID]);
              
              flag = flag && checkIfDone(treeedge);
          }
          if (flag) {
              count += 1;
              done[netID] = true;
              return;
        	}
          else {
            sttrees[netID].x_min = xGrid - 1;
            sttrees[netID].x_max = 0;
            sttrees[netID].y_min = yGrid - 1;
            sttrees[netID].y_max = 0;

            for (int i = 0; i < deg; i++) {
                sttrees[netID].x_min = min(treenodes[i].x, sttrees[netID].x_min);
                sttrees[netID].x_max = max(treenodes[i].x, sttrees[netID].x_max);
                sttrees[netID].y_min = min(treenodes[i].y, sttrees[netID].y_min);
                sttrees[netID].y_max = max(treenodes[i].y, sttrees[netID].y_max);
              
            }
          }
        });
    acc_count += count.reduce();
}

void SPRoute::mazeRouteMSMD(int iter, int expand, float costHeight, int ripup_threshold,
                   int mazeedge_Threshold, bool Ordering, int cost_type, galois::LargeArray<bool>& done,
                   galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>& thread_local_storage) {
  // LOCK = 0;
  float forange;
  // allocate memory for distance and parent and pop_heap
  h_costTable = (float*)calloc(40 * hCapacity, sizeof(float));
  v_costTable = (float*)calloc(40 * vCapacity, sizeof(float));

  forange = 40 * hCapacity;

  if (cost_type == 2) {
    for (int i = 0; i < forange; i++) {
      if (i < hCapacity - 1)
        h_costTable[i] =
            costHeight / (exp((float)(hCapacity - i - 1) * LOGIS_COF) + 1) + 1;
      else
        h_costTable[i] =
            costHeight / (exp((float)(hCapacity - i - 1) * LOGIS_COF) + 1) + 1 +
            costHeight / slope * (i - hCapacity);
    }
    forange = 40 * vCapacity;
    for (int i = 0; i < forange; i++) {
      if (i < vCapacity - 1)
        v_costTable[i] =
            costHeight / (exp((float)(vCapacity - i - 1) * LOGIS_COF) + 1) + 1;
      else
        v_costTable[i] =
            costHeight / (exp((float)(vCapacity - i - 1) * LOGIS_COF) + 1) + 1 +
            costHeight / slope * (i - vCapacity);
    }
  } else {

    for (int i = 0; i < forange; i++) {
      if (i < hCapacity)
        h_costTable[i] =
            costHeight / (exp((float)(hCapacity - i) * LOGIS_COF) + 1) + 1;
      else
        h_costTable[i] =
            costHeight / (exp((float)(hCapacity - i) * LOGIS_COF) + 1) + 1 +
            costHeight / slope * (i - hCapacity);
    }
    forange = 40 * vCapacity;
    for (int i = 0; i < forange; i++) {
      if (i < vCapacity)
        v_costTable[i] =
            costHeight / (exp((float)(vCapacity - i) * LOGIS_COF) + 1) + 1;
      else
        v_costTable[i] =
            costHeight / (exp((float)(vCapacity - i) * LOGIS_COF) + 1) + 1 +
            costHeight / slope * (i - vCapacity);
    }
  }
  // cout << " i = vCap:" << v_costTable[vCapacity-1] << " " <<
  // v_costTable[vCapacity] << " " << v_costTable[vCapacity+1] << endl;

  /*forange = yGrid*xGrid;
  for(int i=0; i<forange; i++)
  {
      pop_heap2[i] = false;
  } //Michael*/

  if (Ordering) {
    StNetOrder();
    // printf("order?\n");
  }

  // for(nidRPC=0; nidRPC<numValidNets; nidRPC++)//parallelize
  PerThread_PQ perthread_pq;
  PerThread_Vec perthread_vec;

  galois::GAccumulator<int> total_ripups;
  galois::GReduceMax<int> max_ripups;
  total_ripups.reset();
  max_ripups.reset();

  galois::GAccumulator<uint32_t> count;
  galois::GAccumulator<uint32_t> total_queue_cnt;

  // galois::runtime::profileVtune( [&] (void) {
  /*std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(net_shuffle.begin(), net_shuffle.end(), g);

  galois::do_all(galois::iterate(net_shuffle), */
  // galois::for_each(galois::iterate(0, numValidNets),
  //        [&] (const auto nidRPC, auto& ctx)
  galois::do_all(
      galois::iterate(0, numValidNets),
      [&](const auto nidRPC) {
        int grid, netID;

        // maze routing for multi-source, multi-destination
        bool hypered, enter;
        int i, j, deg, edgeID, n1, n2, n1x, n1y, n2x, n2y, ymin, ymax, xmin,
            xmax, curX, curY, crossX, crossY, tmpX, tmpY, tmpi, min_x, min_y,
            num_edges;
        int regionX1, regionX2, regionY1, regionY2;
        int ind1, tmpind, gridsX[XRANGE], gridsY[YRANGE], tmp_gridsX[XRANGE],
            tmp_gridsY[YRANGE];
        int endpt1, endpt2, A1, A2, B1, B2, C1, C2, D1, D2, cnt, cnt_n1n2;
        int edge_n1n2, edge_n1A1, edge_n1A2, edge_n1C1, edge_n1C2, edge_A1A2,
            edge_C1C2;
        int edge_n2B1, edge_n2B2, edge_n2D1, edge_n2D2, edge_B1B2, edge_D1D2;
        int E1x, E1y, E2x, E2y;
        int tmp_grid;
        int preX, preY, origENG, edgeREC;

        float tmp, tmp_cost;
        TreeEdge *treeedges, *treeedge;
        TreeNode* treenodes;

        bool* pop_heap2 = thread_local_storage.getLocal()->pop_heap2;

        float** d1    = thread_local_storage.getLocal()->d1_p;
        bool** HV     = thread_local_storage.getLocal()->HV_p;
        bool** hyperV = thread_local_storage.getLocal()->hyperV_p;
        bool** hyperH = thread_local_storage.getLocal()->hyperH_p;

        short** parentX1 = thread_local_storage.getLocal()->parentX1_p;
        short** parentX3 = thread_local_storage.getLocal()->parentX3_p;
        short** parentY1 = thread_local_storage.getLocal()->parentY1_p;
        short** parentY3 = thread_local_storage.getLocal()->parentY3_p;

        int** corrEdge = thread_local_storage.getLocal()->corrEdge_p;

        OrderNetEdge* netEO = thread_local_storage.getLocal()->netEO_p;

        bool** inRegion = thread_local_storage.getLocal()->inRegion_p;

        local_pq pq1 = perthread_pq.get();
        local_vec v2 = perthread_vec.get();

        /*for(i=0; i<yGrid*xGrid; i++)
        {
            pop_heap2[i] = false;
        } */

        // memset(inRegion_alloc, 0, xGrid * yGrid * sizeof(bool));
        /*for(int i=0; i<yGrid; i++)
        {
            for(int j=0; j<xGrid; j++)
                inRegion[i][j] = false;
        }*/
        // printf("hyperV[153][134]: %d %d %d\n", hyperV[153][134],
        // parentY1[153][134], parentX3[153][134]); printf("what is
        // happening?\n");

        if (Ordering) {
          netID = treeOrderCong[nidRPC].treeIndex;
        } else {
          netID = nidRPC;
        }
        if (done[netID])
          	return;
        
        deg = sttrees[netID].deg;

        origENG = expand;

        netedgeOrderDec(netID, netEO);

        treeedges = sttrees[netID].edges;
        treenodes = sttrees[netID].nodes;
        // loop for all the tree edges (2*deg-3)
        num_edges = 2 * deg - 3;

        bool flag = true;
            
        for (int edgeID = 0; edgeID < num_edges; edgeID++) {
          treeedge = &(treeedges[edgeID]);
          flag = flag && checkIfDone(treeedge);
        }

        if (done[netID]) {
          cout << "this net is done, should not be here" << netID << " " << flag <<  endl;
          exit(1);
        }

        if (flag) {
          count += 1;
          done[netID] = true;
          return;
        }
        //cout << "running: " << nets[netID]->name << endl;
        for (edgeREC = 0; edgeREC < num_edges; edgeREC++) {
          edgeID   = netEO[edgeREC].edgeID;
          treeedge = &(treeedges[edgeID]);

          n1            = treeedge->n1;
          n2            = treeedge->n2;
          n1x           = treenodes[n1].x;
          n1y           = treenodes[n1].y;
          n2x           = treenodes[n2].x;
          n2y           = treenodes[n2].y;
          treeedge->len = ADIFF(n2x, n1x) + ADIFF(n2y, n1y);

          if (treeedge->len >
              mazeedge_Threshold) // only route the non-degraded edges (len>0)
          {
            // enter = newRipupCheck(treeedge, n1x, n1y, n2x, n2y,
            // ripup_threshold, netID, edgeID);
            enter =
                newRipupCheck_atomic(treeedge, ripup_threshold, netID, edgeID);

            // ripup the routing for the edge
            if (enter) {
              /*pre_length = treeedge->route.routelen;
              for(int i = 0; i < pre_length; i++)
              {
                  pre_gridsY[i] = treeedge->route.gridsY[i];
                  pre_gridsX[i] = treeedge->route.gridsX[i];
                  //printf("i %d x %d y %d\n", i, pre_gridsX[i], pre_gridsY[i]);
              }*/
              // if(netID == 252163 && edgeID == 51)
              //    printf("netID %d edgeID %d src %d %d dst %d %d\n", netID,
              //    edgeID, n1x, n1y, n2x, n2y);
              if (n1y <= n2y) {
                ymin = n1y;
                ymax = n2y;
              } else {
                ymin = n2y;
                ymax = n1y;
              }

              if (n1x <= n2x) {
                xmin = n1x;
                xmax = n2x;
              } else {
                xmin = n2x;
                xmax = n1x;
              }

              int enlarge =
                  min(origENG,
                      (iter / 6 + 3) *
                          treeedge->route
                              .routelen); // michael, this was global variable
              regionX1 = max(0, xmin - enlarge);
              regionX2 = min(xGrid - 1, xmax + enlarge);
              regionY1 = max(0, ymin - enlarge);
              regionY2 = min(yGrid - 1, ymax + enlarge);

              // initialize d1[][] and d2[][] as BIG_INT
              for (i = regionY1; i <= regionY2; i++) {
                for (j = regionX1; j <= regionX2; j++) {
                  d1[i][j] = BIG_INT;
                  /*d2[i][j] = BIG_INT;
                  hyperH[i][j] = false;
                  hyperV[i][j] = false;*/
                }
              }
              // memset(hyperH, 0, xGrid * yGrid * sizeof(bool));
              // memset(hyperV, 0, xGrid * yGrid * sizeof(bool));
              for (i = regionY1; i <= regionY2; i++) {
                for (j = regionX1; j <= regionX2; j++) {
                  hyperH[i][j] = false;
                }
              }
              for (i = regionY1; i <= regionY2; i++) {
                for (j = regionX1; j <= regionX2; j++) {
                  hyperV[i][j] = false;
                }
              }
              // TODO: use seperate loops

              // setup heap1, heap2 and initialize d1[][] and d2[][] for all the
              // grids on the two subtrees
              setupHeap(netID, edgeID, pq1, v2, regionX1, regionX2, regionY1,
                        regionY2, d1, corrEdge, inRegion);
              // TODO: use std priority queue
              // while loop to find shortest path
              ind1 = (pq1.top().d1_p - &d1[0][0]);
              pq1.pop();
              curX = ind1 % xGrid;
              curY = ind1 / xGrid;

              for (local_vec::iterator ii = v2.begin(); ii != v2.end(); ii++) {
                pop_heap2[*ii] = true;
              }
              float curr_d1;
              while (pop_heap2[ind1] ==
                     false) // stop until the grid position been popped out from
                            // both heap1 and heap2
              {
                // relax all the adjacent grids within the enlarged region for
                // source subtree

                // if(PRINT) printf("curX curY %d %d, (%d, %d), (%d, %d),
                // pq1.size: %d\n", curX, curY, regionX1, regionX2, regionY1,
                // regionY2, pq1.size()); if(curX == 102 && curY == 221)
                // exit(1);
                curr_d1 = d1[curY][curX];
                if (curr_d1 != 0) {
                  if (HV[curY][curX]) {
                    preX = parentX1[curY][curX];
                    preY = parentY1[curY][curX];
                  } else {
                    preX = parentX3[curY][curX];
                    preY = parentY3[curY][curX];
                  }
                } else {
                  preX = curX;
                  preY = curY;
                }

                // left
                if (curX > regionX1) {
                  grid = curY * (xGrid - 1) + curX - 1; //usage left-curr
                  tmpX = curX - 1; // the left neighbor
                  if ((preY == curY) || (curr_d1 == 0)) {
                    tmp = curr_d1 +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];
                  } else {
                    if (curX < regionX2 - 1) {
                      tmp_grid = curY * (xGrid - 1) + curX; //usage curr-right
                      tmp_cost =
                          d1[curY][curX + 1] +
                          h_costTable[h_edges[tmp_grid].usage +
                                      h_edges[tmp_grid].red +
                                      (int)(L * h_edges[tmp_grid].last_usage)];

                      if (tmp_cost < curr_d1 + VIA &&
                          d1[curY][tmpX] >
                              tmp_cost +
                                  h_costTable
                                      [h_edges[grid].usage + h_edges[grid].red +
                                       (int)(L * h_edges[grid].last_usage)]) {
                        hyperH[curY][curX] = true; // Michael
                      }
                    }
                    tmp = curr_d1 + VIA +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];
                  }
                  // if(LOCK)  h_edges[grid].releaseLock();

                  if (d1[curY][tmpX] >
                      tmp && h_edges[grid].cap > 0) // left neighbor been put into heap1 but needs update
                  {
                    d1[curY][tmpX]       = tmp;
                    parentX3[curY][tmpX] = curX;
                    parentY3[curY][tmpX] = curY;
                    HV[curY][tmpX]       = false;
                    pq1.push({&(d1[curY][tmpX]), tmp});
                    total_queue_cnt += 1;
                  }
                }
                // right
                if (curX < regionX2) {
                  grid = curY * (xGrid - 1) + curX;

                  tmpX = curX + 1; // the right neighbor
                  if ((preY == curY) || (curr_d1 == 0)) {
                    tmp = curr_d1 +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];
                  } else {
                    if (curX > regionX1 + 1) {
                      tmp_grid = curY * (xGrid - 1) + curX - 1;
                      tmp_cost =
                          d1[curY][curX - 1] +
                          h_costTable[h_edges[tmp_grid].usage +
                                      h_edges[tmp_grid].red +
                                      (int)(L * h_edges[tmp_grid].last_usage)];

                      if (tmp_cost < curr_d1 + VIA &&
                          d1[curY][tmpX] >
                              tmp_cost +
                                  h_costTable
                                      [h_edges[grid].usage + h_edges[grid].red +
                                       (int)(L * h_edges[grid].last_usage)]) {
                        hyperH[curY][curX] = true;
                      }
                    }
                    tmp = curr_d1 + VIA +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];
                  }

                  if (d1[curY][tmpX] > tmp && h_edges[grid].cap > 0) // right neighbor been put into
                                            // heap1 but needs update, hard stop for OBS
                  {
                    d1[curY][tmpX]       = tmp;
                    parentX3[curY][tmpX] = curX;
                    parentY3[curY][tmpX] = curY;
                    HV[curY][tmpX]       = false;
                    pq1.push({&(d1[curY][tmpX]), tmp});
                    total_queue_cnt += 1;
                  }
                }
                // bottom
                if (curY > regionY1) {
                  grid = (curY - 1) * xGrid + curX;

                  tmpY = curY - 1; // the bottom neighbor
                  if ((preX == curX) || (curr_d1 == 0)) {
                    tmp = curr_d1 +
                          v_costTable[v_edges[grid].usage + v_edges[grid].red +
                                      (int)(L * v_edges[grid].last_usage)];
                  } else {
                    if (curY < regionY2 - 1) {
                      tmp_grid = curY * xGrid + curX;
                      tmp_cost =
                          d1[curY + 1][curX] +
                          v_costTable[v_edges[tmp_grid].usage +
                                      v_edges[tmp_grid].red +
                                      (int)(L * v_edges[tmp_grid].last_usage)];

                      if (tmp_cost < curr_d1 + VIA &&
                          d1[tmpY][curX] >
                              tmp_cost +
                                  v_costTable
                                      [v_edges[grid].usage + v_edges[grid].red +
                                       (int)(L * v_edges[grid].last_usage)]) {
                        hyperV[curY][curX] = true;
                      }
                    }
                    tmp = curr_d1 + VIA +
                          v_costTable[v_edges[grid].usage + v_edges[grid].red +
                                      (int)(L * v_edges[grid].last_usage)];
                  }

                  if (d1[tmpY][curX] > tmp && v_edges[grid].cap > 0) // bottom neighbor been put into
                                            // heap1 but needs update
                  {
                    d1[tmpY][curX]       = tmp;
                    parentX1[tmpY][curX] = curX;
                    parentY1[tmpY][curX] = curY;
                    HV[tmpY][curX]       = true;
                    pq1.push({&(d1[tmpY][curX]), tmp});
                    total_queue_cnt += 1;
                  }
                }
                // top
                if (curY < regionY2) {
                  grid = curY * xGrid + curX;
                  tmpY = curY + 1; // the top neighbor

                  if ((preX == curX) || (curr_d1 == 0)) {
                    tmp = curr_d1 +
                          v_costTable[v_edges[grid].usage + v_edges[grid].red +
                                      (int)(L * v_edges[grid].last_usage)];
                  } else {
                    if (curY > regionY1 + 1) {
                      tmp_grid = (curY - 1) * xGrid + curX;
                      tmp_cost =
                          d1[curY - 1][curX] +
                          v_costTable[v_edges[tmp_grid].usage +
                                      v_edges[tmp_grid].red +
                                      (int)(L * v_edges[tmp_grid].last_usage)];

                      if (tmp_cost < curr_d1 + VIA &&
                          d1[tmpY][curX] >
                              tmp_cost +
                                  v_costTable
                                      [v_edges[grid].usage + v_edges[grid].red +
                                       (int)(L * v_edges[grid].last_usage)]) {
                        hyperV[curY][curX] = true;
                      }
                    }
                    tmp = curr_d1 + VIA +
                          v_costTable[v_edges[grid].usage + v_edges[grid].red +
                                      (int)(L * v_edges[grid].last_usage)];
                  }
                  if (d1[tmpY][curX] >
                      tmp && v_edges[grid].cap > 0) // top neighbor been put into heap1 but needs update
                  {
                    d1[tmpY][curX]       = tmp;
                    parentX1[tmpY][curX] = curX;
                    parentY1[tmpY][curX] = curY;
                    HV[tmpY][curX]       = true;
                    pq1.push({&(d1[tmpY][curX]), tmp});
                    total_queue_cnt += 1;
                  }
                }

                // update ind1 and ind2 for next loop, Michael: need to check if
                // it is up-to-date value.
                float d1_push;
                do {
                  if(pq1.size() <= 0) {
                    cout << "ERROR: priority queue empty! " << endl; 
                    cout << "netid: " << netID << " netName: " << nets[netID]->name << " edgeid: " << edgeID << endl;
                    bool n1_steiner = (n1 < deg)? true: false;
                    bool n2_steiner = (n2 < deg)? true: false;
                    cout << n1_steiner << " " << n2_steiner << endl;
                    cout << n1x << "," << n1y << " " << n2x << "," << n2y << endl;
                    exit(1);
                  }
                  ind1    = pq1.top().d1_p - &d1[0][0];
                  d1_push = pq1.top().d1_push;
                  pq1.pop();
                  
                  curX = ind1 % xGrid;
                  curY = ind1 / xGrid;
                  
                } while (d1_push != d1[curY][curX]);
              } // while loop

              for (local_vec::iterator ii = v2.begin(); ii != v2.end(); ii++)
                pop_heap2[*ii] = false;

              crossX = ind1 % xGrid;
              crossY = ind1 / xGrid;

              cnt  = 0;
              curX = crossX;
              curY = crossY;
              while (d1[curY][curX] != 0) // loop until reach subtree1
              {

                /*if(cnt > 2000) {
                    cerr << "Y: " << curY <<" X:" << curX << " hyperH: " <<
                hyperH[curY][curX]; cerr << " hyperV:" << hyperV[curY][curX] <<
                " HV: " << HV[curY][curX]; cerr << " d1: " << d1[curY][curX] <<
                endl; cerr << " deadloop return!" << endl; reInitTree(netID);
                    return;
                }*/

                hypered = false;
                if (cnt != 0) {
                  if (curX != tmpX && hyperH[curY][curX]) {
                    curX    = 2 * curX - tmpX;
                    hypered = true;
                  }
                  // printf("hyperV[153][134]: %d\n", hyperV[curY][curX]);
                  if (curY != tmpY && hyperV[curY][curX]) {
                    curY    = 2 * curY - tmpY;
                    hypered = true;
                  }
                }
                tmpX = curX;
                tmpY = curY;
                if (!hypered) {
                  if (HV[tmpY][tmpX]) {
                    curY = parentY1[tmpY][tmpX];
                  } else {
                    curX = parentX3[tmpY][tmpX];
                  }
                }

                tmp_gridsX[cnt] = curX;
                tmp_gridsY[cnt] = curY;
                cnt++;
              }
              // reverse the grids on the path

              for (i = 0; i < cnt; i++) {
                tmpind    = cnt - 1 - i;
                gridsX[i] = tmp_gridsX[tmpind];
                gridsY[i] = tmp_gridsY[tmpind];
              }
              // add the connection point (crossX, crossY)
              gridsX[cnt] = crossX;
              gridsY[cnt] = crossY;
              cnt++;

              curX     = crossX;
              curY     = crossY;
              cnt_n1n2 = cnt;

              // change the tree structure according to the new routing for the
              // tree edge find E1 and E2, and the endpoints of the edges they
              // are on
              E1x = gridsX[0];
              E1y = gridsY[0];
              E2x = gridsX[cnt_n1n2 - 1];
              E2y = gridsY[cnt_n1n2 - 1];

              edge_n1n2 = edgeID;
              // if(netID == 252163 && edgeID == 51)
              //    printf("E1x: %d, E1y: %d, E2x: %d, E2y %d length: %d\n",
              //    E1x, E1y, E2x, E2y, cnt_n1n2);

              // (1) consider subtree1
              if (n1 >= deg && (E1x != n1x || E1y != n1y))
              // n1 is not a pin and E1!=n1, then make change to subtree1,
              // otherwise, no change to subtree1
              {
                // find the endpoints of the edge E1 is on
                endpt1 = treeedges[corrEdge[E1y][E1x]].n1;
                endpt2 = treeedges[corrEdge[E1y][E1x]].n2;

                // find A1, A2 and edge_n1A1, edge_n1A2
                if (treenodes[n1].nbr[0] == n2) {
                  A1        = treenodes[n1].nbr[1];
                  A2        = treenodes[n1].nbr[2];
                  edge_n1A1 = treenodes[n1].edge[1];
                  edge_n1A2 = treenodes[n1].edge[2];
                } else if (treenodes[n1].nbr[1] == n2) {
                  A1        = treenodes[n1].nbr[0];
                  A2        = treenodes[n1].nbr[2];
                  edge_n1A1 = treenodes[n1].edge[0];
                  edge_n1A2 = treenodes[n1].edge[2];
                } else {
                  A1        = treenodes[n1].nbr[0];
                  A2        = treenodes[n1].nbr[1];
                  edge_n1A1 = treenodes[n1].edge[0];
                  edge_n1A2 = treenodes[n1].edge[1];
                }

                if (endpt1 == n1 ||
                    endpt2 == n1) // E1 is on (n1, A1) or (n1, A2)
                {
                  // if E1 is on (n1, A2), switch A1 and A2 so that E1 is always
                  // on (n1, A1)
                  if (endpt1 == A2 || endpt2 == A2) {
                    tmpi      = A1;
                    A1        = A2;
                    A2        = tmpi;
                    tmpi      = edge_n1A1;
                    edge_n1A1 = edge_n1A2;
                    edge_n1A2 = tmpi;
                  }

                  // update route for edge (n1, A1), (n1, A2)
                  updateRouteType1(treenodes, n1, A1, A2, E1x, E1y, treeedges,
                                   edge_n1A1, edge_n1A2);
                  // update position for n1
                  treenodes[n1].x = E1x;
                  treenodes[n1].y = E1y;
                }    // if E1 is on (n1, A1) or (n1, A2)
                else // E1 is not on (n1, A1) or (n1, A2), but on (C1, C2)
                {
                  C1        = endpt1;
                  C2        = endpt2;
                  edge_C1C2 = corrEdge[E1y][E1x];

                  // update route for edge (n1, C1), (n1, C2) and (A1, A2)
                  updateRouteType2(treenodes, n1, A1, A2, C1, C2, E1x, E1y,
                                   treeedges, edge_n1A1, edge_n1A2, edge_C1C2);
                  // update position for n1
                  treenodes[n1].x = E1x;
                  treenodes[n1].y = E1y;
                  // update 3 edges (n1, A1)->(C1, n1), (n1, A2)->(n1, C2), (C1,
                  // C2)->(A1, A2)
                  edge_n1C1               = edge_n1A1;
                  treeedges[edge_n1C1].n1 = C1;
                  treeedges[edge_n1C1].n2 = n1;
                  edge_n1C2               = edge_n1A2;
                  treeedges[edge_n1C2].n1 = n1;
                  treeedges[edge_n1C2].n2 = C2;
                  edge_A1A2               = edge_C1C2;
                  treeedges[edge_A1A2].n1 = A1;
                  treeedges[edge_A1A2].n2 = A2;
                  // update nbr and edge for 5 nodes n1, A1, A2, C1, C2
                  // n1's nbr (n2, A1, A2)->(n2, C1, C2)
                  treenodes[n1].nbr[0]  = n2;
                  treenodes[n1].edge[0] = edge_n1n2;
                  treenodes[n1].nbr[1]  = C1;
                  treenodes[n1].edge[1] = edge_n1C1;
                  treenodes[n1].nbr[2]  = C2;
                  treenodes[n1].edge[2] = edge_n1C2;
                  // A1's nbr n1->A2
                  for (i = 0; i < 3; i++) {
                    if (treenodes[A1].nbr[i] == n1) {
                      treenodes[A1].nbr[i]  = A2;
                      treenodes[A1].edge[i] = edge_A1A2;
                      break;
                    }
                  }
                  // A2's nbr n1->A1
                  for (i = 0; i < 3; i++) {
                    if (treenodes[A2].nbr[i] == n1) {
                      treenodes[A2].nbr[i]  = A1;
                      treenodes[A2].edge[i] = edge_A1A2;
                      break;
                    }
                  }
                  // C1's nbr C2->n1
                  for (i = 0; i < 3; i++) {
                    if (treenodes[C1].nbr[i] == C2) {
                      treenodes[C1].nbr[i]  = n1;
                      treenodes[C1].edge[i] = edge_n1C1;
                      break;
                    }
                  }
                  // C2's nbr C1->n1
                  for (i = 0; i < 3; i++) {
                    if (treenodes[C2].nbr[i] == C1) {
                      treenodes[C2].nbr[i]  = n1;
                      treenodes[C2].edge[i] = edge_n1C2;
                      break;
                    }
                  }

                } // else E1 is not on (n1, A1) or (n1, A2), but on (C1, C2)
              }   // n1 is not a pin and E1!=n1

              // (2) consider subtree2

              if (n2 >= deg && (E2x != n2x || E2y != n2y))
              // n2 is not a pin and E2!=n2, then make change to subtree2,
              // otherwise, no change to subtree2
              {
                // find the endpoints of the edge E1 is on
                endpt1 = treeedges[corrEdge[E2y][E2x]].n1;
                endpt2 = treeedges[corrEdge[E2y][E2x]].n2;

                // find B1, B2
                if (treenodes[n2].nbr[0] == n1) {
                  B1        = treenodes[n2].nbr[1];
                  B2        = treenodes[n2].nbr[2];
                  edge_n2B1 = treenodes[n2].edge[1];
                  edge_n2B2 = treenodes[n2].edge[2];
                } else if (treenodes[n2].nbr[1] == n1) {
                  B1        = treenodes[n2].nbr[0];
                  B2        = treenodes[n2].nbr[2];
                  edge_n2B1 = treenodes[n2].edge[0];
                  edge_n2B2 = treenodes[n2].edge[2];
                } else {
                  B1        = treenodes[n2].nbr[0];
                  B2        = treenodes[n2].nbr[1];
                  edge_n2B1 = treenodes[n2].edge[0];
                  edge_n2B2 = treenodes[n2].edge[1];
                }

                if (endpt1 == n2 ||
                    endpt2 == n2) // E2 is on (n2, B1) or (n2, B2)
                {
                  // if E2 is on (n2, B2), switch B1 and B2 so that E2 is always
                  // on (n2, B1)
                  if (endpt1 == B2 || endpt2 == B2) {
                    tmpi      = B1;
                    B1        = B2;
                    B2        = tmpi;
                    tmpi      = edge_n2B1;
                    edge_n2B1 = edge_n2B2;
                    edge_n2B2 = tmpi;
                  }

                  // update route for edge (n2, B1), (n2, B2)
                  updateRouteType1(treenodes, n2, B1, B2, E2x, E2y, treeedges,
                                   edge_n2B1, edge_n2B2);

                  // update position for n2
                  treenodes[n2].x = E2x;
                  treenodes[n2].y = E2y;
                }    // if E2 is on (n2, B1) or (n2, B2)
                else // E2 is not on (n2, B1) or (n2, B2), but on (D1, D2)
                {
                  D1        = endpt1;
                  D2        = endpt2;
                  edge_D1D2 = corrEdge[E2y][E2x];

                  // update route for edge (n2, D1), (n2, D2) and (B1, B2)
                  updateRouteType2(treenodes, n2, B1, B2, D1, D2, E2x, E2y,
                                   treeedges, edge_n2B1, edge_n2B2, edge_D1D2);
                  // update position for n2
                  treenodes[n2].x = E2x;
                  treenodes[n2].y = E2y;
                  // update 3 edges (n2, B1)->(D1, n2), (n2, B2)->(n2, D2), (D1,
                  // D2)->(B1, B2)
                  edge_n2D1               = edge_n2B1;
                  treeedges[edge_n2D1].n1 = D1;
                  treeedges[edge_n2D1].n2 = n2;
                  edge_n2D2               = edge_n2B2;
                  treeedges[edge_n2D2].n1 = n2;
                  treeedges[edge_n2D2].n2 = D2;
                  edge_B1B2               = edge_D1D2;
                  treeedges[edge_B1B2].n1 = B1;
                  treeedges[edge_B1B2].n2 = B2;
                  // update nbr and edge for 5 nodes n2, B1, B2, D1, D2
                  // n1's nbr (n1, B1, B2)->(n1, D1, D2)
                  treenodes[n2].nbr[0]  = n1;
                  treenodes[n2].edge[0] = edge_n1n2;
                  treenodes[n2].nbr[1]  = D1;
                  treenodes[n2].edge[1] = edge_n2D1;
                  treenodes[n2].nbr[2]  = D2;
                  treenodes[n2].edge[2] = edge_n2D2;
                  // B1's nbr n2->B2
                  for (i = 0; i < 3; i++) {
                    if (treenodes[B1].nbr[i] == n2) {
                      treenodes[B1].nbr[i]  = B2;
                      treenodes[B1].edge[i] = edge_B1B2;
                      break;
                    }
                  }
                  // B2's nbr n2->B1
                  for (i = 0; i < 3; i++) {
                    if (treenodes[B2].nbr[i] == n2) {
                      treenodes[B2].nbr[i]  = B1;
                      treenodes[B2].edge[i] = edge_B1B2;
                      break;
                    }
                  }
                  // D1's nbr D2->n2
                  for (i = 0; i < 3; i++) {
                    if (treenodes[D1].nbr[i] == D2) {
                      treenodes[D1].nbr[i]  = n2;
                      treenodes[D1].edge[i] = edge_n2D1;
                      break;
                    }
                  }
                  // D2's nbr D1->n2
                  for (i = 0; i < 3; i++) {
                    if (treenodes[D2].nbr[i] == D1) {
                      treenodes[D2].nbr[i]  = n2;
                      treenodes[D2].edge[i] = edge_n2D2;
                      break;
                    }
                  }
                } // else E2 is not on (n2, B1) or (n2, B2), but on (D1, D2)
              }   // n2 is not a pin and E2!=n2

              // update route for edge (n1, n2) and edge usage

              // printf("update route? %d %d\n", netID, num_edges);
              if (treeedges[edge_n1n2].route.type == MAZEROUTE) {
                free(treeedges[edge_n1n2].route.gridsX);
                free(treeedges[edge_n1n2].route.gridsY);
              }
              treeedges[edge_n1n2].route.gridsX =
                  (short*)calloc(cnt_n1n2, sizeof(short));
              treeedges[edge_n1n2].route.gridsY =
                  (short*)calloc(cnt_n1n2, sizeof(short));
              treeedges[edge_n1n2].route.type     = MAZEROUTE;
              treeedges[edge_n1n2].route.routelen = cnt_n1n2 - 1;
              treeedges[edge_n1n2].len = ADIFF(E1x, E2x) + ADIFF(E1y, E2y);
              treeedges[edge_n1n2].n_ripups += 1;
              total_ripups += 1;
              max_ripups.update(treeedges[edge_n1n2].n_ripups);

              for (i = 0; i < cnt_n1n2; i++) {
                // printf("cnt_n1n2: %d\n", cnt_n1n2);
                treeedges[edge_n1n2].route.gridsX[i] = gridsX[i];
                treeedges[edge_n1n2].route.gridsY[i] = gridsY[i];
              }

              // update edge usage

              for (i = 0; i < cnt_n1n2 - 1; i++) {
                if (gridsX[i] == gridsX[i + 1]) // a vertical edge
                {
                  min_y = min(gridsY[i], gridsY[i + 1]);
                  v_edges[min_y * xGrid + gridsX[i]].usage.fetch_add(
                      (short int)1);
 
                } else /// if(gridsY[i]==gridsY[i+1])// a horizontal edge
                {
                  min_x = min(gridsX[i], gridsX[i + 1]);
                  h_edges[gridsY[i] * (xGrid - 1) + min_x].usage.fetch_add(
                      (short int)1);
 
                }
              }

              if (checkRoute2DTree(netID)) {
                reInitTree(netID);
                return;
              }
            } // congested route
          }   // maze routing
        }     // loop edgeID
      },
      // galois::wl<galois::worklists::ParaMeter<>>(),
      galois::steal(),
      // galois::chunk_size<64>(),
      galois::loopname("net-level parallelism")); // galois::do_all
  
  if(verbose_ > none)
    printf("total ripups: %d max ripups: %d\n", total_ripups.reduce(),
         max_ripups.reduce());
  //}, "mazeroute vtune function");

    acc_count += count.reduce();

  if(verbose_ > none) {
    std::cout <<" count: " << count.reduce() << " acc_count: " << acc_count << " remaining: " << numValidNets - acc_count << std::endl;
    std::cout << "total queue cnt: " << total_queue_cnt.reduce() << endl;
  }
  free(h_costTable);
  free(v_costTable);
}




} //namespace sproute