#ifndef DETPART_H
#define DETPART_H

#include <vector>
#include "galois/LargeArray.h"



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

Bool newRipupCheck_atomic_deterministic(TreeEdge* treeedge, int ripup_threshold, int netID,
                          int edgeID, Edge* h_edges_local, Edge* v_edges_local){ 
  short *gridsX, *gridsY;
  int i, grid, ymin, xmin;
  Bool needRipup = FALSE;
  int break_edge = 0;

  if (treeedge->len == 0) {
    return (FALSE);
  } // not ripup for degraded edge
  // std::cout << " atomic ripup" << std::endl;
  if (treeedge->route.type == MAZEROUTE) {
    gridsX = treeedge->route.gridsX;
    gridsY = treeedge->route.gridsY;
    for (i = 0; i < treeedge->route.routelen; i++) {
        if (gridsX[i] == gridsX[i + 1]) // a vertical edge
        {
            ymin          = min(gridsY[i], gridsY[i + 1]);
            grid          = ymin * xGrid + gridsX[i];
            int old_usage = v_edges[grid].usage;

            if(old_usage + v_edges[grid].red >= vCapacity - ripup_threshold) {
            
                break_edge = i;
                needRipup  = TRUE;
                v_edges_local[grid].usage.fetch_sub((short unsigned)1, std::memory_order_relaxed);
                //v_edges[grid].usage.fetch_sub((short unsigned)1, std::memory_order_relaxed);
            }
            if (needRipup)
                break;

      } 
      else if (gridsY[i] == gridsY[i + 1]) // a horizontal edge
      {
            xmin          = min(gridsX[i], gridsX[i + 1]);
            grid          = gridsY[i] * (xGrid - 1) + xmin;
            int old_usage = h_edges[grid].usage;

            if(old_usage + h_edges[grid].red >= hCapacity - ripup_threshold){
                break_edge = i;
                needRipup  = TRUE;
                h_edges_local[grid].usage.fetch_sub((short unsigned)1, std::memory_order_relaxed);
                //h_edges[grid].usage.fetch_sub((short unsigned)1, std::memory_order_relaxed);
            }
            if (needRipup)
                break;
      }
      else {
          cout << "ripup check error: neither horizontal or vertical" << endl;
          exit(1);
      }
    }

    if (needRipup) {
      for (i = 0; i < treeedge->route.routelen; i++) {
        if (i == break_edge)
          continue;
        if (gridsX[i] == gridsX[i + 1]) // a vertical edge
        {
            ymin = min(gridsY[i], gridsY[i + 1]);
            //v_edges[ymin * xGrid + gridsX[i]].usage.fetch_sub(
            //  (short unsigned)1, std::memory_order_relaxed);
			v_edges_local[ymin * xGrid + gridsX[i]].usage.fetch_sub(
              (short unsigned)1, std::memory_order_relaxed);
        	
		} 
        else /// if(gridsY[i]==gridsY[i+1])// a horizontal edge
        {
            xmin = min(gridsX[i], gridsX[i + 1]);
            //h_edges[gridsY[i] * (xGrid - 1) + xmin].usage.fetch_sub(
            //  (short unsigned)1, std::memory_order_relaxed);
			h_edges_local[gridsY[i] * (xGrid - 1) + xmin].usage.fetch_sub(
              (short unsigned)1, std::memory_order_relaxed);
        }
      }
      return (TRUE);
    } else {
      return (FALSE);
    }
  } else {
    printf("route type is not maze, netID %d\n", netID);
    fflush(stdout);
    printEdge(netID, edgeID);

    exit(1);
  }
}

void checkOverflowRemaining(galois::LargeArray<bool> &done) {

    int overflow = 0;
    for(int netID = 0; netID < numValidNets; netID++) {
        if(!done[netID]) {

            int deg = sttrees[netID].deg;
            auto treeedges = sttrees[netID].edges;
            int num_edges = 2 * deg - 3;

            for (int edgeID = 0; edgeID < num_edges; edgeID++) {
          	    auto treeedge = &(treeedges[edgeID]);
                auto gridsX = treeedge->route.gridsX;
                auto gridsY = treeedge->route.gridsY;
                for (int i = 0; i < treeedge->route.routelen; i++) {
                    if (gridsX[i] == gridsX[i + 1]) // a vertical edge
                    {   
                        int ymin = min(gridsY[i], gridsY[i + 1]);
                        int grid = ymin * xGrid + gridsX[i];
                        if(v_edges[grid].usage > v_edges[grid].cap)	
                            overflow += v_edges[grid].usage - v_edges[grid].cap;
                    } 
                    else if (gridsY[i] == gridsY[i + 1]) {// a horizontal edge 
                        int xmin = min(gridsX[i], gridsX[i + 1]);
                        int grid = gridsY[i] * (xGrid - 1) + xmin;
                        if(h_edges[grid].usage > h_edges[grid].cap)  
                            overflow += h_edges[grid].usage - h_edges[grid].cap;
                    }
                    else {
                        cout << "checkOverflowRemaining" << endl;
                        exit(1);
                    }
                }
            }
        }
    }
    cout << "The overflow from the remaining nets are: " << overflow << endl;
}

void mazeRouteMSMDDetPart(int iter, int expand, float costHeight,
                          int ripup_threshold, int mazeedge_Threshold,
                          Bool Ordering, int cost_type, int parts,
                          std::vector<std::vector<int>>& vecParts, galois::LargeArray<bool> &done) {
  // LOCK = 0;
  float forange;
  std::cout << "inside mazeRouteMSMDDetPart\n";
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
      pop_heap2[i] = FALSE;
  } //Michael*/

  if (Ordering) {
    StNetOrder();
  }

  galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>
      thread_local_storage{};
  // for(nidRPC=0; nidRPC<numValidNets; nidRPC++)//parallelize
  PerThread_PQ perthread_pq;
  PerThread_Vec perthread_vec;
  PRINT = 0;
  galois::GAccumulator<int> total_ripups;
  galois::GReduceMax<int> max_ripups;
  total_ripups.reset();
  max_ripups.reset();

  // galois::runtime::profileVtune( [&] (void) {
  /*std::random_device rd;
  std::mt19937 g(rd());
  std::shuffle(net_shuffle.begin(), net_shuffle.end(), g);

  galois::do_all(galois::iterate(net_shuffle), */
  // galois::for_each(galois::iterate(0, numValidNets),
  //        [&] (const auto nidRPC, auto& ctx)

  // decalring local h_edge and v_edges
  Edge* h_edges_local;
  Edge* v_edges_local;

  // galois::StatTimer roundtimer1("round");
  // roundtimer1.start();
  h_edges_local = (Edge*)calloc(((xGrid - 1) * yGrid), sizeof(Edge));
  v_edges_local = (Edge*)calloc((xGrid * (yGrid - 1)), sizeof(Edge));

  // partition nets
  std::vector<galois::InsertBag<int>> bag(parts);
  // galois::InsertBag<int> toProcess_h_edges;
  // galois::InsertBag<int> toProcess_v_edges;

  galois::LargeArray<bool> inBag_h_edges;
  galois::LargeArray<bool> inBag_v_edges;

  inBag_h_edges.allocateBlocked((xGrid - 1) * yGrid + 1);
  inBag_v_edges.allocateBlocked(xGrid * (yGrid - 1) + 1);

  //	std::cout << "nets: " << numValidNets << std::endl;

  // int factor = numValidNets/parts +1;
  // galois::do_all(galois::iterate(0, numValidNets),
  //[&](const auto nidRPC) {

  //	for(int nidRPC = 0;nidRPC < numValidNets; nidRPC++){
  // int idx = nidRPC%parts;
  //			int idx = nidRPC/factor ;
  // bag[idx].push(nidRPC);
  //	});

  galois::do_all(galois::iterate((int)0, (xGrid - 1) * yGrid), [&](int c) {
    inBag_h_edges[c]       = false;
    h_edges_local[c].usage = 0;
    // h_edges_local[c].red = 0;
    // h_edges_local[c].last_usage = 0;
  });

  galois::do_all(galois::iterate((int)0, xGrid * (yGrid - 1)), [&](int c) {
    inBag_v_edges[c]       = false;
    v_edges_local[c].usage = 0;
    // v_edges_local[c].red = 0;
    // v_edges_local[c].last_usage = 0;
  });

	galois::GAccumulator<uint32_t> count;

  //	std::cout <<"size: " << (xGrid - 1) * yGrid << std::endl;
  //	roundtimer1.stop();
  galois::StatTimer roundtimer1("round");
  roundtimer1.start();
  // std::cout << "time: " << roundtimer1.get() << std::endl;
  for (int idx = 0; idx < parts; idx++) {
    galois::do_all(
        //     galois::iterate(0, numValidNets),
        galois::iterate(vecParts[idx]),
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
              pop_heap2[i] = FALSE;
          } */

          // memset(inRegion_alloc, 0, xGrid * yGrid * sizeof(bool));
          /*for(int i=0; i<yGrid; i++)
          {
              for(int j=0; j<xGrid; j++)
                  inRegion[i][j] = FALSE;
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
              // enter =
              // newRipupCheck_atomic(treeedge, ripup_threshold, netID, edgeID);

              enter = newRipupCheck_atomic_deterministic(
                  treeedge, ripup_threshold, netID, edgeID, h_edges_local,
                  v_edges_local);

              // ripup the routing for the edge
              if (enter) {
                /*pre_length = treeedge->route.routelen;
                for(int i = 0; i < pre_length; i++)
                {
                    pre_gridsY[i] = treeedge->route.gridsY[i];
                    pre_gridsX[i] = treeedge->route.gridsX[i];
                    //printf("i %d x %d y %d\n", i, pre_gridsX[i],
                pre_gridsY[i]);
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
                    hyperH[i][j] = FALSE;
                    hyperV[i][j] = FALSE;*/
                  }
                }
                // memset(hyperH, 0, xGrid * yGrid * sizeof(bool));
                // memset(hyperV, 0, xGrid * yGrid * sizeof(bool));
                for (i = regionY1; i <= regionY2; i++) {
                  for (j = regionX1; j <= regionX2; j++) {
                    hyperH[i][j] = FALSE;
                  }
                }
                for (i = regionY1; i <= regionY2; i++) {
                  for (j = regionX1; j <= regionX2; j++) {
                    hyperV[i][j] = FALSE;
                  }
                }
                // TODO: use seperate loops

                // setup heap1, heap2 and initialize d1[][] and d2[][] for all
                // the grids on the two subtrees
                setupHeap(netID, edgeID, pq1, v2, regionX1, regionX2, regionY1,
                          regionY2, d1, corrEdge, inRegion);
                // TODO: use std priority queue
                // while loop to find shortest path
                ind1 = (pq1.top().d1_p - &d1[0][0]);
                pq1.pop();
                curX = ind1 % xGrid;
                curY = ind1 / xGrid;

                for (local_vec::iterator ii = v2.begin(); ii != v2.end();
                     ii++) {
                  pop_heap2[*ii] = TRUE;
                }
                float curr_d1;
                while (pop_heap2[ind1] ==
                       FALSE) // stop until the grid position been popped out
                              // from both heap1 and heap2
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
                    grid = curY * (xGrid - 1) + curX - 1;
                    tmpX = curX - 1; // the left neighbor
                    if ((preY == curY) || (curr_d1 == 0)) {
                      tmp =
                          curr_d1 +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];
                    } else {
                      if (curX < regionX2 - 1) {
                        tmp_grid = curY * (xGrid - 1) + curX;
                        tmp_cost =
                            d1[curY][curX + 1] +
                            h_costTable[h_edges[tmp_grid].usage +
                                        h_edges[tmp_grid].red +
                                        (int)(L *
                                              h_edges[tmp_grid].last_usage)];

                        /*tmp_cost =
      d1[curY][curX + 1] +
      h_costTable[h_edges[tmp_grid].usage + h_edges_local[tmp_grid].usage +
                  h_edges[tmp_grid].red + h_edges_local[tmp_grid].red +
                  (int)(L * (h_edges[tmp_grid].last_usage +
      h_edges_local[tmp_grid].last_usage))];
*/
                        if (tmp_cost < curr_d1 + VIA &&
                            d1[curY][tmpX] >
                                tmp_cost +
                                    h_costTable[h_edges[grid].usage +
                                                h_edges[grid].red +
                                                (int)(L * h_edges[grid]
                                                              .last_usage)]) {
                          hyperH[curY][curX] = TRUE; // Michael
                        }

                        /*if (tmp_cost < curr_d1 + VIA &&
     d1[curY][tmpX] >
         tmp_cost +
             h_costTable
                 [h_edges[grid].usage + h_edges_local[grid].usage +
 h_edges[grid].red + h_edges_local[grid].red + (int)(L *
 (h_edges[grid].last_usage + h_edges_local[grid].last_usage))]) {
   hyperH[curY][curX] = TRUE; // Michael
 }*/
                      }
                      tmp =
                          curr_d1 + VIA +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];

                      // tmp = curr_d1 + VIA +
                      //    h_costTable[h_edges[grid].usage +
                      //    h_edges_local[grid].usage +  h_edges[grid].red +
                      //    h_edges_local[grid].red +
                      //              (int)(L * (h_edges[grid].last_usage +
                      //              h_edges_local[grid].last_usage))];
                    }
                    // if(LOCK)  h_edges[grid].releaseLock();

                    if (d1[curY][tmpX] > tmp) // left neighbor been put into
                                              // heap1 but needs update
                    {
                      d1[curY][tmpX]       = tmp;
                      parentX3[curY][tmpX] = curX;
                      parentY3[curY][tmpX] = curY;
                      HV[curY][tmpX]       = FALSE;
                      pq1.push({&(d1[curY][tmpX]), tmp});
                    }
                  }
                  // right
                  if (curX < regionX2) {
                    grid = curY * (xGrid - 1) + curX;

                    tmpX = curX + 1; // the right neighbor
                    if ((preY == curY) || (curr_d1 == 0)) {
                      tmp =
                          curr_d1 +
                          h_costTable[h_edges[grid].usage + h_edges[grid].red +
                                      (int)(L * h_edges[grid].last_usage)];

                                    /*		tmp = curr_d1 +
                          h_costTable[h_edges[grid].usage + h_edges_local[grid].usage + h_edges[grid].red + h_edges_local[grid].red +
                                      (int)(L * (h_edges[grid].last_usage + h_edges_local[grid].last_usage))];
								*/	} else {
                                      if (curX > regionX1 + 1) {
                                        tmp_grid =
                                            curY * (xGrid - 1) + curX - 1;
                                        tmp_cost =
                                            d1[curY][curX - 1] +
                                            h_costTable
                                                [h_edges[tmp_grid].usage +
                                                 h_edges[tmp_grid].red +
                                                 (int)(L * h_edges[tmp_grid]
                                                               .last_usage)];
                                        /*
                                        tmp_cost =
                      d1[curY][curX - 1] +
                      h_costTable[h_edges[tmp_grid].usage +
                      h_edges_local[tmp_grid].usage + h_edges[tmp_grid].red +
                      h_edges_local[tmp_grid].red + (int)(L *
                      (h_edges[tmp_grid].last_usage +
                      h_edges_local[tmp_grid].last_usage))];
*/
                                        if (tmp_cost < curr_d1 + VIA &&
                                            d1[curY][tmpX] >
                                                tmp_cost +
                                                    h_costTable
                                                        [h_edges[grid].usage +
                                                         h_edges[grid].red +
                                                         (int)(L *
                                                               h_edges[grid]
                                                                   .last_usage)]) {
                                          hyperH[curY][curX] = TRUE;
                                        }
                                        /*	if (tmp_cost < curr_d1 + VIA &&
                          d1[curY][tmpX] >
                              tmp_cost +
                                  h_costTable
                                      [h_edges[grid].usage +
                      h_edges_local[grid].usage + h_edges[grid].red +
                      h_edges_local[grid].red + (int)(L *
                      (h_edges[grid].last_usage +
                      h_edges_local[grid].last_usage))]) { hyperH[curY][curX] =
                      TRUE;
                      }*/
                                      }
                                      tmp = curr_d1 + VIA +
                                            h_costTable[h_edges[grid].usage +
                                                        h_edges[grid].red +
                                                        (int)(L *
                                                              h_edges[grid]
                                                                  .last_usage)];

                                    /*	tmp = curr_d1 + VIA +
                          h_costTable[h_edges[grid].usage + h_edges_local[grid].usage + h_edges[grid].red + h_edges_local[grid].red +
                                      (int)(L * (h_edges[grid].last_usage + h_edges_local[grid].last_usage))];
                 */ }

                                    if (d1[curY][tmpX] >
                                        tmp) // right neighbor been put into
                                             // heap1 but needs update
                                    {
                                      d1[curY][tmpX]       = tmp;
                                      parentX3[curY][tmpX] = curX;
                                      parentY3[curY][tmpX] = curY;
                                      HV[curY][tmpX]       = FALSE;
                                      pq1.push({&(d1[curY][tmpX]), tmp});
                                    }
                  }
                  // bottom
                  if (curY > regionY1) {
                    grid = (curY - 1) * xGrid + curX;

                    tmpY = curY - 1; // the bottom neighbor
                    if ((preX == curX) || (curr_d1 == 0)) {
                      tmp =
                          curr_d1 +
                          v_costTable[v_edges[grid].usage + v_edges[grid].red +
                                      (int)(L * v_edges[grid].last_usage)];

                                        /*	tmp = curr_d1 +
                          v_costTable[v_edges[grid].usage + v_edges_local[grid].usage + v_edges[grid].red + v_edges_local[grid].red +
                                      (int)(L * (v_edges[grid].last_usage + v_edges_local[grid].last_usage))];
							*/		} else {
                                          if (curY < regionY2 - 1) {
                                            tmp_grid = curY * xGrid + curX;
                                            tmp_cost =
                                                d1[curY + 1][curX] +
                                                v_costTable
                                                    [v_edges[tmp_grid].usage +
                                                     v_edges[tmp_grid].red +
                                                     (int)(L *
                                                           v_edges[tmp_grid]
                                                               .last_usage)];

                                            /*tmp_cost =
                          d1[curY + 1][curX] +
                          v_costTable[v_edges[tmp_grid].usage +
                          v_edges_local[tmp_grid].usage + v_edges[tmp_grid].red
                          + v_edges_local[tmp_grid].red + (int)(L *
                          (v_edges[tmp_grid].last_usage +
                          v_edges_local[tmp_grid].last_usage))];
*/
                                            if (tmp_cost < curr_d1 + VIA &&
                                                d1[tmpY][curX] >
                                                    tmp_cost +
                                                        v_costTable
                                                            [v_edges[grid]
                                                                 .usage +
                                                             v_edges[grid].red +
                                                             (int)(L *
                                                                   v_edges[grid]
                                                                       .last_usage)]) {
                                              hyperV[curY][curX] = TRUE;
                                            }

                                            /*if (tmp_cost < curr_d1 + VIA &&
                          d1[tmpY][curX] >
                              tmp_cost +
                                  v_costTable
                                      [v_edges[grid].usage +
                      v_edges_local[grid].usage  +  v_edges[grid].red +
                      v_edges_local[grid].red + (int)(L *
                      (v_edges[grid].last_usage +
                      v_edges_local[grid].last_usage))]) { hyperV[curY][curX] =
                      TRUE;
                      }*/
                                          }
                                          tmp =
                                              curr_d1 + VIA +
                                              v_costTable
                                                  [v_edges[grid].usage +
                                                   v_edges[grid].red +
                                                   (int)(L * v_edges[grid]
                                                                 .last_usage)];
                                        /*
										tmp = curr_d1 + VIA +
                          v_costTable[v_edges[grid].usage + v_edges_local[grid].usage   + v_edges[grid].red + v_edges_local[grid].red+
                                      (int)(L * (v_edges[grid].last_usage + v_edges_local[grid].last_usage))];
                */  }

                                        if (d1[tmpY][curX] >
                                            tmp) // bottom neighbor been put
                                                 // into heap1 but needs update
                                        {
                                          d1[tmpY][curX]       = tmp;
                                          parentX1[tmpY][curX] = curX;
                                          parentY1[tmpY][curX] = curY;
                                          HV[tmpY][curX]       = TRUE;
                                          pq1.push({&(d1[tmpY][curX]), tmp});
                                        }
                  }
                  // top
                  if (curY < regionY2) {
                    grid = curY * xGrid + curX;
                    tmpY = curY + 1; // the top neighbor

                    if ((preX == curX) || (curr_d1 == 0)) {
                      tmp =
                          curr_d1 +
                          v_costTable[v_edges[grid].usage + v_edges[grid].red +
                                      (int)(L * v_edges[grid].last_usage)];

                                        /*tmp = curr_d1 +
                          v_costTable[v_edges[grid].usage + v_edges_local[grid].usage + v_edges[grid].red + v_edges_local[grid].red +
                                      (int)(L * (v_edges[grid].last_usage + v_edges_local[grid].last_usage))];
						*/			} else {
                                          if (curY > regionY1 + 1) {
                                            tmp_grid =
                                                (curY - 1) * xGrid + curX;
                                            tmp_cost =
                                                d1[curY - 1][curX] +
                                                v_costTable
                                                    [v_edges[tmp_grid].usage +
                                                     v_edges[tmp_grid].red +
                                                     (int)(L *
                                                           v_edges[tmp_grid]
                                                               .last_usage)];

                                            /*	tmp_cost =
                              d1[curY - 1][curX] +
                              v_costTable[v_edges[tmp_grid].usage +
                              v_edges_local[tmp_grid].usage +
                                          v_edges[tmp_grid].red +
                              v_edges_local[tmp_grid].red + (int)(L *
                              (v_edges[tmp_grid].last_usage +
                              v_edges_local[tmp_grid].last_usage))];
            */
                                            if (tmp_cost < curr_d1 + VIA &&
                                                d1[tmpY][curX] >
                                                    tmp_cost +
                                                        v_costTable
                                                            [v_edges[grid]
                                                                 .usage +
                                                             v_edges[grid].red +
                                                             (int)(L *
                                                                   v_edges[grid]
                                                                       .last_usage)]) {
                                              hyperV[curY][curX] = TRUE;
                                            }
                                            /*if (tmp_cost < curr_d1 + VIA &&
                          d1[tmpY][curX] >
                              tmp_cost +
                                  v_costTable
                                      [v_edges[grid].usage +
                      v_edges_local[grid].usage + v_edges[grid].red +
                      v_edges_local[grid].red + (int)(L *
                      (v_edges[grid].last_usage +
                      v_edges_local[grid].last_usage))]) { hyperV[curY][curX] =
                      TRUE;
                      }*/
                                          }
                                          tmp =
                                              curr_d1 + VIA +
                                              v_costTable
                                                  [v_edges[grid].usage +
                                                   v_edges[grid].red +
                                                   (int)(L * v_edges[grid]
                                                                 .last_usage)];
                                            /*tmp = curr_d1 + VIA +
                          v_costTable[v_edges[grid].usage + v_edges_local[grid].usage  + v_edges[grid].red + v_edges_local[grid].red +
                                      (int)(L * (v_edges[grid].last_usage + v_edges_local[grid].last_usage))];
                */  }
                                        if (d1[tmpY][curX] >
                                            tmp) // top neighbor been put into
                                                 // heap1 but needs update
                                        {
                                          d1[tmpY][curX]       = tmp;
                                          parentX1[tmpY][curX] = curX;
                                          parentY1[tmpY][curX] = curY;
                                          HV[tmpY][curX]       = TRUE;
                                          pq1.push({&(d1[tmpY][curX]), tmp});
                                        }
                  }

                  // update ind1 and ind2 for next loop, Michael: need to check
                  // if it is up-to-date value.
                  float d1_push;
                  do {
                    ind1    = pq1.top().d1_p - &d1[0][0];
                    d1_push = pq1.top().d1_push;
                    pq1.pop();
                    curX = ind1 % xGrid;
                    curY = ind1 / xGrid;
                  } while (d1_push != d1[curY][curX]);
                } // while loop

                for (local_vec::iterator ii = v2.begin(); ii != v2.end(); ii++)
                  pop_heap2[*ii] = FALSE;

                crossX = ind1 % xGrid;
                crossY = ind1 / xGrid;

                cnt  = 0;
                curX = crossX;
                curY = crossY;
                while (d1[curY][curX] != 0) // loop until reach subtree1
                {

                  /*if(cnt > 2000) {
                      cerr << "Y: " << curY <<" X:" << curX << " hyperH: " <<
                  hyperH[curY][curX]; cerr << " hyperV:" << hyperV[curY][curX]
                  << " HV: " << HV[curY][curX]; cerr << " d1: " <<
                  d1[curY][curX] << endl; cerr << " deadloop return!" << endl;
                  reInitTree(netID); return;
                  }*/

                  hypered = FALSE;
                  if (cnt != 0) {
                    if (curX != tmpX && hyperH[curY][curX]) {
                      curX    = 2 * curX - tmpX;
                      hypered = TRUE;
                    }
                    // printf("hyperV[153][134]: %d\n", hyperV[curY][curX]);
                    if (curY != tmpY && hyperV[curY][curX]) {
                      curY    = 2 * curY - tmpY;
                      hypered = TRUE;
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

                // change the tree structure according to the new routing for
                // the tree edge find E1 and E2, and the endpoints of the edges
                // they are on
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
                    // if E1 is on (n1, A2), switch A1 and A2 so that E1 is
                    // always on (n1, A1)
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
                                     treeedges, edge_n1A1, edge_n1A2,
                                     edge_C1C2);
                    // update position for n1
                    treenodes[n1].x = E1x;
                    treenodes[n1].y = E1y;
                    // update 3 edges (n1, A1)->(C1, n1), (n1, A2)->(n1, C2),
                    // (C1, C2)->(A1, A2)
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
                    // if E2 is on (n2, B2), switch B1 and B2 so that E2 is
                    // always on (n2, B1)
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
                                     treeedges, edge_n2B1, edge_n2B2,
                                     edge_D1D2);
                    // update position for n2
                    treenodes[n2].x = E2x;
                    treenodes[n2].y = E2y;
                    // update 3 edges (n2, B1)->(D1, n2), (n2, B2)->(n2, D2),
                    // (D1, D2)->(B1, B2)
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

                /*for(i=0; i<pre_length; i++)
                {
                    if(pre_gridsX[i]==pre_gridsX[i+1]) // a vertical edge
                    {
                        if(i != pre_length - 1)
                            min_y = min(pre_gridsY[i], pre_gridsY[i+1]);
                        else
                            min_y = pre_gridsY[i];
                        //v_edges[min_y*xGrid+gridsX[i]].usage += 1;
                        //galois::atomicAdd(v_edges[min_y*xGrid+gridsX[i]].usage,
                (short unsigned)1);
                        //printf("x y %d %d i %d \n", pre_gridsX[i], min_y, i);
                        v_edges[min_y*xGrid+pre_gridsX[i]].usage.fetch_sub((short
                int)1);
                        //if(v_edges[min_y*xGrid+pre_gridsX[i]].usage < 0)
                printf("V negative! %d \n", i);
                    }
                    else ///if(gridsY[i]==gridsY[i+1])// a horizontal edge
                    {
                        if(i != pre_length - 1)
                            min_x = min(pre_gridsX[i], pre_gridsX[i+1]);
                        else
                            min_x = pre_gridsX[i];
                        //h_edges[gridsY[i]*(xGrid-1)+min_x].usage += 1;
                        //galois::atomicAdd(h_edges[gridsY[i]*(xGrid-1)+min_x].usage,
                (short unsigned)1);
                        //printf("x y %d %d i %d\n", min_x, pre_gridsY[i], i);
                        h_edges[pre_gridsY[i]*(xGrid-1)+min_x].usage.fetch_sub((short
                int)1);
                        //if(h_edges[pre_gridsY[i]*(xGrid-1)+min_x].usage < 0)
                printf("H negative! %d \n", i);
                    }
                }*/

                for (i = 0; i < cnt_n1n2 - 1; i++) {
                  if (gridsX[i] == gridsX[i + 1]) // a vertical edge
                  {
                    min_y = min(gridsY[i], gridsY[i + 1]);
                    // v_edges[min_y*xGrid+gridsX[i]].usage += 1;
                    // galois::atomicAdd(v_edges[min_y*xGrid+gridsX[i]].usage,
                    // (short unsigned)1);
                    // v_edges[min_y * xGrid + gridsX[i]].usage.fetch_add(
                    //  (short int)1);
                    v_edges_local[min_y * xGrid + gridsX[i]].usage.fetch_add(
                        (short int)1);

                    // if(!inBag_v_edges[min_y * xGrid + gridsX[i]]){

                    // toProcess_v_edges.push(min_y * xGrid + gridsX[i]);
                    //				inBag_v_edges[min_y * xGrid + gridsX[i]] =
                    // true;
                    //	}
                  } else /// if(gridsY[i]==gridsY[i+1])// a horizontal edge
                  {
                    min_x = min(gridsX[i], gridsX[i + 1]);
                    // h_edges[gridsY[i]*(xGrid-1)+min_x].usage += 1;
                    // galois::atomicAdd(h_edges[gridsY[i]*(xGrid-1)+min_x].usage,
                    // (short unsigned)1);
                    // h_edges[gridsY[i] * (xGrid - 1) + min_x].usage.fetch_add(
                    //  (short int)1);
                    h_edges_local[gridsY[i] * (xGrid - 1) + min_x]
                        .usage.fetch_add((short int)1);

                    // if(!inBag_h_edges[gridsY[i] * (xGrid - 1) + min_x]){

                    // toProcess_h_edges.push(gridsY[i] * (xGrid - 1) + min_x);
                    //					inBag_h_edges[gridsY[i] * (xGrid - 1) + min_x]
                    //= true;
                    //}
                  }
                }
                /*if(LOCK){
                    for(i=0; i<cnt_n1n2-1; i++)
                    {
                        if(gridsX[i]==gridsX[i+1]) // a vertical edge
                        {
                            min_y = min(gridsY[i], gridsY[i+1]);
                            v_edges[min_y*xGrid+gridsX[i]].releaseLock();
                        }
                        else ///if(gridsY[i]==gridsY[i+1])// a horizontal edge
                        {
                            min_x = min(gridsX[i], gridsX[i+1]);
                            h_edges[gridsY[i]*(xGrid-1)+min_x].releaseLock();
                        }
                    }
                }*/
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

    //	galois::StatTimer roundtimer1("round");
    // roundtimer1.start();
    

    // update h_edges

    galois::do_all(galois::iterate((int)0, (xGrid - 1) * yGrid), [&](int c) {
      h_edges[c].usage += h_edges_local[c].usage;
      h_edges_local[c].usage = 0;

      
    });

    // update v_edges
    galois::do_all(galois::iterate((int)0, xGrid * (yGrid - 1)), [&](int c) {
      v_edges[c].usage += v_edges_local[c].usage;
      v_edges_local[c].usage = 0;

    });


  } // end for

  roundtimer1.stop();
  printf("rtimer: %d\n", roundtimer1.get());

  printf("total ripups: %d max ripups: %d\n", total_ripups.reduce(),
         max_ripups.reduce());
  //}, "mazeroute vtune function");
  free(h_costTable);
  free(v_costTable);

  free(h_edges_local);
  free(v_edges_local);

  inBag_h_edges.destroy();
  inBag_h_edges.deallocate();
    acc_count += count.reduce();

	std::cout <<" count: " << count.reduce() << " acc_count: " << acc_count << " remaining: " << numValidNets - acc_count << std::endl;

  inBag_v_edges.destroy();
  inBag_v_edges.deallocate();

  //checkOverflowRemaining(done);

}




#endif