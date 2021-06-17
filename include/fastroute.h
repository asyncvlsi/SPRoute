#ifndef FASTROUTE_H
#define FASTROUTE_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

#include "galois/Galois.h"
#include "galois/Reduction.h"
#include "galois/PriorityQueue.h"
#include "galois/Timer.h"
#include "galois/graphs/Graph.h"
#include "galois/graphs/TypeTraits.h"
#include "galois/substrate/SimpleLock.h"
#include "galois/AtomicHelpers.h"
#include "galois/runtime/Profile.h"

#include "galois/LargeArray.h"

//#include "Lonestar/BFS_SSSP.h"

#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "RSMT.h"
#include "maze.h"
#include "RipUp.h"
#include "utility.h"
#include "route.h"
#include "maze3D.h"
#include "maze_finegrain.h"
#include "maze_finegrain_lateupdate.h"
#include "maze_lock.h"

#include "algo.h"
#include "grGen.h"
#include "detpart.h"
#include "a_star.h"
#include "invalidnets_adj.h"
#include "mempin_adj.h"
#include "rudy.h"
#include "detpart_astar.h"
#include "detpart_astar_data.h"
#include "detpart_astar_local.h"

void readGR(sproute::grGenerator& grGen, Algo algo)
{
	FILE *fp;
    int pinX, pinY, pinL, netID, numPins, pinInd, grid, newnetID, invalid_netID, segcount, minwidth;
    float pinX_in,pinY_in;
    int maxDeg = 0;
    int pinXarray[MAXNETDEG], pinYarray[MAXNETDEG], pinLarray[MAXNETDEG];
    char netName[STRINGLEN];
    Bool remove;
    int numAdjust, cap, reduce, reducedCap;
    int adjust,net;
    int TC;

    net = 0;
    xGrid = grGen.grid.x;
    yGrid = grGen.grid.y;
    numLayers = grGen.grid.z;

    for (int i=0; i<numLayers; i++)
    {
        //vCapacity3D[i] = grGen.vCap.at(i);
        //vCapacity3D[i] = (grGen.vCap.at(i) - GLOBAL_CAP_REDUCTION > 0)?  grGen.vCap.at(i) - GLOBAL_CAP_REDUCTION : grGen.vCap.at(i) ;
        if(grGen.vCap.at(i) != 0)
        {
            //vCapacity3D[i] = ((float) grGen.vCap.at(i) * GLOBAL_CAP_ADJ <= 1)?  1 : grGen.vCap.at(i) * GLOBAL_CAP_ADJ ;
            vCapacity3D[i] = grGen.vCap.at(i);
        }
        else
        {
            vCapacity3D[i] = 0;
        }
        vCapacity+=vCapacity3D[i];
    }

    for (int i=0; i<numLayers; i++)
    {
        //hCapacity3D[i] = grGen.hCap.at(i);
        //hCapacity3D[i] = (grGen.hCap.at(i) - GLOBAL_CAP_REDUCTION > 0)?  grGen.hCap.at(i) - GLOBAL_CAP_REDUCTION : grGen.hCap.at(i) ;
        if(grGen.hCap.at(i) != 0)
        {
            //hCapacity3D[i] = ((float) grGen.hCap.at(i) * GLOBAL_CAP_ADJ <= 1)?  1 : grGen.hCap.at(i) * GLOBAL_CAP_ADJ ;
            hCapacity3D[i] = grGen.hCap.at(i);
        }
        else
        {
            hCapacity3D[i] = 0;
        }
        hCapacity+=hCapacity3D[i];
    }

    xcorner = grGen.gcellBegin.x;
    ycorner = grGen.gcellBegin.y;
    wTile = grGen.gcellSize.x;
    hTile = grGen.gcellSize.y;

    numNets = grGen.numNets;

    numGrids = xGrid*yGrid;
    
    vCapacity_lb = LB*vCapacity;
    hCapacity_lb = LB*hCapacity;
    vCapacity_ub = UB*vCapacity;
    hCapacity_ub = UB*hCapacity;

    printf("\n");
    printf("grid %d %d %d\n", xGrid, yGrid, numLayers);
    for (int i=0;i<numLayers;i++)
    { 
        printf("Layer %d vertical capacity %d, horizontal capacity %d\n", i, vCapacity3D[i], hCapacity3D[i]);
    }
    

    printf("total vertical capacity %d\n", vCapacity);
    printf("total horizontal capacity %d\n", hCapacity);
    printf("num net %d\n", numNets);

    // allocate memory for nets
    nets = (Net**) malloc(numNets*sizeof(Net*));
    invalid_nets = (Net**) malloc(numNets*sizeof(Net*));
    for(int i=0; i<numNets; i++)
    {
        nets[i] = (Net*) malloc(sizeof(Net));
        invalid_nets[i] = (Net*) malloc(sizeof(Net));
    }
    seglistIndex = (int*) malloc(numNets*sizeof(int));
    
    // read nets information from the input file
    segcount = 0;
    newnetID = 0;
    invalid_netID = 0;


    for(int i=0; i<numNets; i++)
    {
        net++;
        strcpy(netName, grGen.grnets.at(i).name.c_str());
        netID = grGen.grnets.at(i).idx;
        numPins = grGen.grnets.at(i).numPins;
        if(numPins > 1000)
            cout << "reading large net: " << netName << " " << numPins << endl;
        //cout << netName << " " << netID << " " << numPins << endl;
        if (1)
        {
            pinInd = 0;
            for(int j=0; j<numPins; j++) {
                //fscanf(fp, "%f	%f	%d\n", &pinX_in, &pinY_in, &pinL);
                pinX_in = grGen.grnets.at(i).pins.at(j).x;
                pinY_in = grGen.grnets.at(i).pins.at(j).y;
                pinL = grGen.grnets.at(i).pins.at(j).z;

                pinX = sproute::find_Gcell(pinX_in, defDB.xGcellBoundaries);
                pinY = sproute::find_Gcell(pinY_in, defDB.yGcellBoundaries);
                

                
                /*int tmp_pinX = (int)((pinX_in-xcorner)/wTile);
                int tmp_pinY = (int)((pinY_in-ycorner)/hTile);
                if(pinX != tmp_pinX || pinY != tmp_pinY)
                {
                	cout << pinX_in << " " << pinY_in << endl;
                	cout << pinX << " " << pinY << endl;
                	cout << tmp_pinX << " " << tmp_pinY << endl;
                	cout << xcorner << " " << ycorner << " " << wTile << " " << hTile << endl;
                	exit(1);
                }*/
                if(strcmp(netName, "Reset") == 0 && pinX == 67 && pinY == 0)
                {
                    cout << "reading Reset:" << endl;
                    cout << "pinX: " << pinX << " pinY: " << pinY << " pinL: " << pinL << " numLayers: " << numLayers << endl;
                }

                if(!(pinX < 0 || pinX > xGrid || pinY < -1 || pinY > yGrid|| pinL > numLayers || pinL<=0))
                {
                    
                    remove = FALSE;
                    for(int k=0; k<pinInd; k++)
                    {
                        if(pinX==pinXarray[k] && pinY==pinYarray[k] && pinL==pinLarray[k])
                        {remove=TRUE; break;}
                    }
                    if(!remove) // the pin is in different grid from other pins
                    {
                        pinXarray[pinInd] = (pinX == xGrid)? (pinX - 1) : pinX;
                        pinYarray[pinInd] = (pinY == yGrid)? (pinY - 1) : pinY;
                        pinLarray[pinInd] = pinL;
                        pinInd++;

                    }
                }
            }
        
            if(pinInd>1) // valid net
            {
                MD = max(MD, pinInd);
                TD += pinInd;        
                strcpy(nets[newnetID]->name, netName);
                nets[newnetID]->netIDorg = netID;
                nets[newnetID]->numPins = numPins;
                nets[newnetID]->deg = pinInd;
                nets[newnetID]->pinX = (short*) malloc(pinInd*sizeof(short));
                nets[newnetID]->pinY = (short*) malloc(pinInd*sizeof(short));
                nets[newnetID]->pinL = (short*) malloc(pinInd*sizeof(short));

                for (int j=0; j<pinInd; j++)
                {
                    nets[newnetID]->pinX[j] = pinXarray[j];
                    nets[newnetID]->pinY[j] = pinYarray[j];
                    nets[newnetID]->pinL[j] = pinLarray[j];
                }
                maxDeg = pinInd>maxDeg?pinInd:maxDeg;
                seglistIndex[newnetID] = segcount;
                newnetID++;
                segcount += 2*pinInd-3; // at most (2*numPins-2) nodes, (2*numPins-3) nets for a net
            } // if valid net
            else
            {
                strcpy(invalid_nets[invalid_netID]->name, netName);
                invalid_nets[invalid_netID]->netIDorg = netID;
                invalid_nets[invalid_netID]->numPins = numPins;
                invalid_nets[invalid_netID]->deg = pinInd;
                invalid_nets[invalid_netID]->pinX = (short*) malloc(pinInd*sizeof(short));
                invalid_nets[invalid_netID]->pinY = (short*) malloc(pinInd*sizeof(short));
                invalid_nets[invalid_netID]->pinL = (short*) malloc(pinInd*sizeof(short));


                invalid_nets[invalid_netID]->pinX[0] = pinXarray[0];
                invalid_nets[invalid_netID]->pinY[0] = pinYarray[0];
                invalid_nets[invalid_netID]->pinL[0] = pinLarray[0];
                
                //cout << "netName: " << netName << " pinInd: " << pinInd << endl;
                //cout << " X Y Z: " << pinXarray[0] << " " << pinYarray[0] << " " << pinLarray[0] << endl;
                invalid_netID++;
            }
        }//if

        if(numPins > 1000)
            cout << "reading large net finished: " << netName << endl;
        if(i%10000 == 0)
            cout << "read " << i << " nets." << endl;
    } // loop i
    printf("the total net number is %d\n\n",net);

    if((pinInd>1))
    {
		seglistIndex[newnetID] = segcount; // the end pointer of the seglist
    }
    numValidNets = newnetID;
    numInvalidNets = invalid_netID;
    
       // allocate memory and initialize for edges
	float* rudy = new float [xGrid * yGrid];
	if(algo == RUDY)
		plot_rudy(rudy, algo);
	
	if(algo == DRC_MAP) {
		plot_drc_map();
        exit(0);
    }
	float avg_pin_den = plot_pin_density(rudy, algo);
	cout << "avg pin density: " << avg_pin_den << endl;
	if(algo == RUDY || algo == PIN_DENSITY || algo == DRC_MAP) {
		exit(0) ;
	}

    h_edges = (Edge*)calloc(((xGrid-1)*yGrid), sizeof(Edge));
    v_edges = (Edge*)calloc((xGrid*(yGrid-1)), sizeof(Edge));

	v_edges3D = (Edge3D*)calloc((numLayers*xGrid*yGrid), sizeof(Edge3D));
	h_edges3D = (Edge3D*)calloc((numLayers*xGrid*yGrid), sizeof(Edge3D));

    //2D edge innitialization
    TC=0;

	for (int k=0; k<numLayers; k++)
	{
		for(int i=0; i<yGrid; i++)
		{
			for(int j=0; j<xGrid-1; j++)
			{
				float local_rudy;
				if(j <= xGrid - 2)
					//local_rudy = max(rudy[i*xGrid + j], rudy[i*xGrid + j + 1]);
					local_rudy = 0.5 * (rudy[i*xGrid + j] + rudy[i*xGrid + j + 1]);
				else 
					local_rudy = rudy[i*xGrid + j];

				int grid3D = i*(xGrid-1)+j+k*(xGrid-1)*yGrid;
				h_edges3D[grid3D].cap = GLOBAL_CAP_ADJ(hCapacity3D[k], local_rudy, k);
				h_edges3D[grid3D].usage = 0;
				h_edges3D[grid3D].red = hCapacity3D[k] - h_edges3D[grid3D].cap;
				
			} 
		}
		for(int i=0; i<yGrid-1; i++)
		{
			for(int j=0; j<xGrid; j++)
			{
				float local_rudy;
				if(i <= yGrid - 2) 
					//local_rudy = max(rudy[i * xGrid + j], rudy[(i + 1)*xGrid + j]);
					local_rudy = 0.5 * (rudy[i * xGrid + j] + rudy[(i + 1)*xGrid + j]);
				else 
					local_rudy = rudy[i*xGrid + j];

				int grid3D = i*xGrid+j+k*xGrid*(yGrid-1);
				v_edges3D[grid3D].cap = GLOBAL_CAP_ADJ(vCapacity3D[k], local_rudy, k);
				v_edges3D[grid3D].usage = 0;
				v_edges3D[grid3D].red = vCapacity3D[k] - v_edges3D[grid3D].cap;
            }
		}
	}

	for(int i=0; i<yGrid; i++)
    {
        for(int j=0; j<xGrid-1; j++)
        {
			int local_cap = 0;
			for(int k=0; k<numLayers; k++) {
				int grid3D = i*(xGrid-1)+j+k*(xGrid-1)*yGrid;
				local_cap += h_edges3D[grid3D].cap;
			}

            int grid2D = i*(xGrid-1)+j;
            //h_edges[grid].cap = hCapacity;
            //TC+=hCapacity;

			h_edges[grid2D].cap = local_cap;
            TC += local_cap;
			h_edges[grid2D].usage = 0;
            h_edges[grid2D].est_usage = 0;
            h_edges[grid2D].red = hCapacity - local_cap;
			h_edges[grid2D].last_usage = 0;

        }
    }

    for(int i=0; i<yGrid-1; i++)
    {
        for(int j=0; j<xGrid; j++)
        {	
			int local_cap = 0;
			for(int k=0; k<numLayers; k++) {
				int grid3D = i*xGrid+j+k*xGrid*(yGrid-1);
				local_cap += v_edges3D[grid3D].cap;
			}

            int grid2D = i*xGrid+j;

            v_edges[grid2D].cap = local_cap;
            TC += local_cap;
            v_edges[grid2D].usage = 0;
            v_edges[grid2D].est_usage = 0;
            v_edges[grid2D].red = vCapacity - local_cap;
			v_edges[grid2D].last_usage=0;

        }
    }

    for(auto capReduction : *(grGen.capReductions_p))
    {

        int x1 = capReduction.x;
        int y1 = capReduction.y;
        int k = capReduction.z; //bottom layer is 0

        if (lefDB.layers.at(k * 2).direction == "HORIZONTAL")//horizontal edge
        {
			float local_rudy;
			if(x1 <= xGrid - 2)
				local_rudy = 0.5 * (rudy[y1*xGrid + x1] + rudy[y1*xGrid + x1 + 1]);
				//local_rudy = max(rudy[y1*xGrid + x1], rudy[y1*xGrid + x1 + 1]);
			else 
				local_rudy = rudy[y1*xGrid + x1];


            grid = y1*(xGrid-1)+x1+k*(xGrid-1)*yGrid;

            reducedCap = GLOBAL_CAP_ADJ(capReduction.newCap, local_rudy, k);

            cap = h_edges3D[grid].cap;
            reduce = cap - reducedCap;
            h_edges3D[grid].cap = reducedCap;
            h_edges3D[grid].red += reduce;
            grid=y1*(xGrid-1)+x1;
            h_edges[grid].cap -= reduce;
            h_edges[grid].red += reduce;

        } 
        else if (lefDB.layers.at(k * 2).direction == "VERTICAL")//vertical edge
        {
			float local_rudy;
			if(y1 <= yGrid - 2) 
				local_rudy = 0.5 * (rudy[y1 * xGrid + x1] + rudy[(y1 + 1)*xGrid + x1]);
				//local_rudy = max(rudy[y1 * xGrid + x1], rudy[(y1 + 1)*xGrid + x1]);
			else 
				local_rudy = rudy[y1*xGrid + x1];

            grid = y1*xGrid+x1+k*xGrid*(yGrid-1);

            reducedCap = GLOBAL_CAP_ADJ(capReduction.newCap, local_rudy, k);
            cap=v_edges3D[grid].cap;
            reduce=cap-reducedCap;
            v_edges3D[grid].cap = reducedCap;
            v_edges3D[grid].red += reduce;
            grid = y1*xGrid+x1;
            v_edges[grid].cap-=reduce;
            v_edges[grid].red += reduce;
            
        }
        else
        {
            cout << "unknown layer direction in cap reduction" << endl;
            exit(1);
        }

    }
	invalidNetsAdj(invalid_nets, xGrid, yGrid);

	/*int tmpX = 145, tmpY = 87;
	int grid2D = tmpY*xGrid+tmpX;
	cout << "145, 87: " << v_edges[grid2D].cap << endl;
	for(int k = 0; k < numLayers; k++) {
		int grid3D = tmpY*xGrid+tmpX+k*xGrid*(yGrid-1);
		cout << " 3D cap: " << v_edges3D[grid3D].cap << endl;
	}*/

	memPinAdj(xGrid, yGrid);

	/*cout << "145, 87: " << v_edges[grid2D].cap << endl;
	for(int k = 0; k < numLayers; k++) {
		int grid3D = tmpY*xGrid+tmpX+k*xGrid*(yGrid-1);
		cout << " 3D cap: " << v_edges3D[grid3D].cap << endl;
	}*/

	treeOrderCong = NULL;
	stopDEC = FALSE;

    seglistCnt = (int*) malloc(numValidNets*sizeof(int));
    seglist = (Segment*) malloc(segcount*sizeof(Segment));
    trees = (Tree*) malloc(numValidNets*sizeof(Tree));
    sttrees = (StTree*) malloc(numValidNets*sizeof(StTree));
    gxs = (DTYPE**) malloc(numValidNets*sizeof(DTYPE*));
    gys = (DTYPE**) malloc(numValidNets*sizeof(DTYPE*));
    gs =  (DTYPE**) malloc(numValidNets*sizeof(DTYPE*));

	gridHV = XRANGE*YRANGE;
	gridH = (xGrid-1)*yGrid;
	gridV = xGrid*(yGrid-1);
    //cout << " gridH " << gridH << " gridV " << gridV << endl;
    for (int k=0; k<numLayers; k++)
	{
		gridHs[k] = k*gridH;
		gridVs[k] = k * gridV;
	}
    
    MaxDegree=MD;
    
    printf("# valid nets: %d\n", numValidNets);
    printf("# segments: %d\n", segcount);
    printf("maxDeg:     %d\n", maxDeg);
    printf("\nDone getting input\n");
    printf("MD: %d, AD: %.2f, #nets: %d, #routed nets: %d\n", MD, (float)TD/newnetID, numNets, newnetID); 
    printf("TC is %d\n",TC);  


	/*parentX1 = (short**)calloc(yGrid, sizeof(short*));
    parentY1 = (short**)calloc(yGrid, sizeof(short*));
    parentX3 = (short**)calloc(yGrid, sizeof(short*));
    parentY3 = (short**)calloc(yGrid, sizeof(short*));

   
    for(i=0; i<yGrid; i++)
    {
        parentX1[i] = (short*)calloc(xGrid, sizeof(short));
        parentY1[i] = (short*)calloc(xGrid, sizeof(short));
        parentX3[i] = (short*)calloc(xGrid, sizeof(short));
        parentY3[i] = (short*)calloc(xGrid, sizeof(short));
    }*/
    
    
    /*pop_heap2 = (Bool*)calloc(yGrid*XRANGE, sizeof(Bool));

    // allocate memory for priority queue
    heap1 = (float**)calloc((yGrid*xGrid), sizeof(float*));
    heap2 = (float**)calloc((yGrid*xGrid), sizeof(float*));*/
        
	sttreesBK = NULL;
}

void printUndone(galois::LargeArray<bool>& done) {
	cout << "undone nets: " << endl;
	for(int netID=0; netID<numValidNets; netID++) {
		if(!done[netID])
			cout << netID << " " << nets[netID]->name << endl;
	}
}





#endif
