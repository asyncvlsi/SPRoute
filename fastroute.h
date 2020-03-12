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
#include "galois/graphs/TypeTraits.h"
#include "galois/substrate/SimpleLock.h"
#include "galois/AtomicHelpers.h"
#include "galois/runtime/Profile.h"

#include "galois/LargeArray.h"
#include "llvm/Support/CommandLine.h"

#include "Lonestar/BoilerPlate.h"
#include "Lonestar/BFS_SSSP.h"

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


#include "grGen.h"

/*static const char* appName = "parallel fastroute on galois";
static const char* desc =
    "parallel fastroute on galois";
static const char* url = "parallel fastroute on galois";*/




void readGR(parser::grGenerator& grGen)
{
	FILE *fp;
    int i, j, k;
    int pinX, pinY, pinL, netID, numPins, pinInd, grid, newnetID, invalid_netID, segcount, minwidth;
    float pinX_in,pinY_in;
    int maxDeg = 0;
    int pinXarray[MAXNETDEG], pinYarray[MAXNETDEG], pinLarray[MAXNETDEG];
    char netName[STRINGLEN];
    Bool remove;
    int numAdjust, x1, x2, y1, y2, l1, l2, cap, reduce, reducedCap;
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
            vCapacity3D[i] = GLOBAL_CAP_ADJ(grGen.vCap.at(i));
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
            hCapacity3D[i] = GLOBAL_CAP_ADJ(grGen.hCap.at(i));
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
    for (i=0;i<numLayers;i++)
    { 
        printf("Layer %d vertical capacity %d, horizontal capacity %d\n", i, vCapacity3D[i], hCapacity3D[i]);

    }
    

    printf("total vertical capacity %d\n", vCapacity);
    printf("total horizontal capacity %d\n", hCapacity);
    printf("num net %d\n", numNets);

    // allocate memory for nets
    nets = (Net**) malloc(numNets*sizeof(Net*));
    invalid_nets = (Net**) malloc(numNets*sizeof(Net*));
    for(i=0; i<numNets; i++)
    {
        nets[i] = (Net*) malloc(sizeof(Net));
        invalid_nets[i] = (Net*) malloc(sizeof(Net));
    }
    seglistIndex = (int*) malloc(numNets*sizeof(int));
    
    // read nets information from the input file
    segcount = 0;
    newnetID = 0;
    invalid_netID = 0;


    for(i=0; i<numNets; i++)
    {
        net++;
        strcpy(netName, grGen.grnets.at(i).name.c_str());
        netID = grGen.grnets.at(i).idx;
        numPins = grGen.grnets.at(i).numPins;
        if(numPins > 1000)
            cout << "reading large net: " << netName << endl;
        //cout << netName << " " << netID << " " << numPins << endl;
        if (1)
        {
            pinInd = 0;
            for(j=0; j<numPins; j++)
            {
                //fscanf(fp, "%f	%f	%d\n", &pinX_in, &pinY_in, &pinL);
                pinX_in = grGen.grnets.at(i).pins.at(j).x;
                pinY_in = grGen.grnets.at(i).pins.at(j).y;
                pinL = grGen.grnets.at(i).pins.at(j).z;

                pinX = parser::find_Gcell(pinX_in, defDB.xGcellBoundaries);
                pinY = parser::find_Gcell(pinY_in, defDB.yGcellBoundaries);
                

                
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
                if(strcmp(netName, "net71037") == 0)
                {
                    cout << "reading net71037:" << endl;
                    cout << "pinX: " << pinX << " pinY: " << pinY << " pinL: " << pinL << " numLayers: " << numLayers << endl;
                }

                if(!(pinX < 0 || pinX > xGrid || pinY < -1 || pinY > yGrid|| pinL > numLayers || pinL<=0))
                {
                    
                    remove = FALSE;
                    for(k=0; k<pinInd; k++)
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

                for (j=0; j<pinInd; j++)
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

    h_edges = (Edge*)calloc(((xGrid-1)*yGrid), sizeof(Edge));
    v_edges = (Edge*)calloc((xGrid*(yGrid-1)), sizeof(Edge));

	v_edges3D = (Edge3D*)calloc((numLayers*xGrid*yGrid), sizeof(Edge3D));
	h_edges3D = (Edge3D*)calloc((numLayers*xGrid*yGrid), sizeof(Edge3D));

    //2D edge innitialization
    TC=0;
    for(i=0; i<yGrid; i++)
    {
        for(j=0; j<xGrid-1; j++)
        {
            grid = i*(xGrid-1)+j;
            h_edges[grid].cap = hCapacity;
            TC+=hCapacity;
            h_edges[grid].usage = 0;
            h_edges[grid].est_usage = 0;
            h_edges[grid].red=0;
			h_edges[grid].last_usage=0;

            h_edges[grid].max_ripups=0;
            h_edges[grid].max_have_rippedups=0;
            h_edges[grid].ripups_cur_round=false;


        }
    }
    for(i=0; i<yGrid-1; i++)
    {
        for(j=0; j<xGrid; j++)
        {
            grid = i*xGrid+j;
            v_edges[grid].cap = vCapacity;
            TC+=vCapacity;
            v_edges[grid].usage = 0;
            v_edges[grid].est_usage = 0;
            v_edges[grid].red=0;
			v_edges[grid].last_usage=0;

            v_edges[grid].max_ripups=0;
            v_edges[grid].max_have_rippedups=0;
            v_edges[grid].ripups_cur_round=false;

        }
    }

	for (k=0; k<numLayers; k++)
	{
		for(i=0; i<yGrid; i++)
		{
			for(j=0; j<xGrid-1; j++)
			{
				grid = i*(xGrid-1)+j+k*(xGrid-1)*yGrid;
				h_edges3D[grid].cap = hCapacity3D[k];
				h_edges3D[grid].usage = 0;
				h_edges3D[grid].red = 0;
				
			} 
		}
		for(i=0; i<yGrid-1; i++)
		{
			for(j=0; j<xGrid; j++)
			{
				grid = i*xGrid+j+k*xGrid*(yGrid-1);
				v_edges3D[grid].cap = vCapacity3D[k];
				v_edges3D[grid].usage = 0;
				v_edges3D[grid].red = 0;
            }
		}
	}
    

    // modify the capacity of edges according to the input file

	//numAdjust = defDB.capReductions.size();
    //fscanf(fp, "%d\n", &numAdjust);
    //printf("num of Adjust is \n", numAdjust);

    //int numAdjust = grGen.numAdjust;
    /*for(int invalid_netID = 0; invalid_netID < numInvalidNets; invalid_netID++)
    {
        if(invalid_nets[invalid_netID]->numPins == 1)
            continue;

        int x = invalid_nets[invalid_netID]->pinX[0];
        int y = invalid_nets[invalid_netID]->pinY[0];
        int z = invalid_nets[invalid_netID]->pinL[0]; // start from 1

        if(z != 1)
        {
            cout << "some invalid nets are not on layer 0! " << invalid_nets[invalid_netID]->name  << " " << x << " " << y << " " << z << endl;
            exit(1);
        }
        string direction = lefDB.layers.at(2 * z).direction; // z's next level, layer 2
        if(direction == "HORIZONTAL")
        {
            if(x > 0)
            {
                int x1 = x - 1;
                int y1 = y;
                int k = z; //  z's next level

                grid = y1*(xGrid-1)+x1+k*(xGrid-1)*yGrid;
                if(h_edges3D[grid].cap > 1)
                {
                    reducedCap = h_edges3D[grid].cap - 1;

                    reduce=1;
                    h_edges3D[grid].cap=reducedCap;
                    h_edges3D[grid].red=reduce;
                    grid=y1*(xGrid-1)+x1;
                    h_edges[grid].cap-=reduce;
                    h_edges[grid].red += reduce;
                }
            }
            if(x < xGrid - 1)
            {
                int x1 = x;
                int y1 = y;
                int k = z; //  z's next level

                grid = y1*(xGrid-1)+x1+k*(xGrid-1)*yGrid;
                if(h_edges3D[grid].cap > 1)
                {
                    reducedCap = h_edges3D[grid].cap - 1;

                    reduce=1;
                    h_edges3D[grid].cap=reducedCap;
                    h_edges3D[grid].red=reduce;
                    grid=y1*(xGrid-1)+x1;
                    h_edges[grid].cap-=reduce;
                    h_edges[grid].red += reduce;
                }
            }
        }
        else if(direction == "VERTICAL")
        {
            if(y > 0)
            {
                int x1 = x;
                int y1 = y - 1;
                int k = z; //  z's next level

                grid = y1*xGrid+x1+k*xGrid*(yGrid-1);
                if(v_edges3D[grid].cap > 1)
                {
                    reducedCap = v_edges3D[grid].cap - 1;

                    reduce = 1;
                    v_edges3D[grid].cap=reducedCap;
                    v_edges3D[grid].red=reduce;
                    grid = y1*xGrid+x1;
                    v_edges[grid].cap-=reduce;
                    v_edges[grid].red += reduce;
                }

            }
            if(y < yGrid - 1)
            {
                int x1 = x;
                int y1 = y;
                int k = z; //  z's next level

                grid = y1*xGrid+x1+k*xGrid*(yGrid-1);

                if(v_edges3D[grid].cap > 1)
                {
                    reducedCap = v_edges3D[grid].cap - 1;

                    reduce = 1;
                    v_edges3D[grid].cap=reducedCap;
                    v_edges3D[grid].red=reduce;
                    grid = y1*xGrid+x1;
                    v_edges[grid].cap-=reduce;
                    v_edges[grid].red += reduce;
                }
            }
        }
        else
        {
            cout << "UNKNOWN DIRECTION of layer in adjustment" << endl;
            exit(1);
        }
    }*/



    //for(int i = 0; i < defDB.capReductions.size(); i++)
    for(auto capReduction : *(grGen.capReductions_p))
    {

        x1 = capReduction.x;
        y1 = capReduction.y;
        k = capReduction.z;


        if (lefDB.layers.at(k * 2).direction == "HORIZONTAL")//horizontal edge
        {
            grid = y1*(xGrid-1)+x1+k*(xGrid-1)*yGrid;

            reducedCap = GLOBAL_CAP_ADJ(capReduction.newCap);
            cap=h_edges3D[grid].cap;
            reduce=cap-reducedCap;
            h_edges3D[grid].cap=reducedCap;
            h_edges3D[grid].red=reduce;
            grid=y1*(xGrid-1)+x1;
            h_edges[grid].cap-=reduce;
            h_edges[grid].red += reduce;
            

        } 
        else if (lefDB.layers.at(k * 2).direction == "VERTICAL")//vertical edge
        {
            grid = y1*xGrid+x1+k*xGrid*(yGrid-1);

            reducedCap = GLOBAL_CAP_ADJ(capReduction.newCap);
            cap=v_edges3D[grid].cap;
            reduce=cap-reducedCap;
            v_edges3D[grid].cap=reducedCap;
            v_edges3D[grid].red=reduce;
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

    /*int tmpX = 338;
    int tmpY = 794;
    int grid2D = tmpY*(xGrid-1)+tmpX;
    cout << "2D cap: " << h_edges[grid2D].cap << " 2D grid: " << grid2D << endl;
    for(int layer = 0; layer < numLayers; layer++)
    {
        int grid3D = tmpY*(xGrid-1)+tmpX+layer*(xGrid-1)*yGrid;
        cout << "3D cap: " << layer << " " << h_edges3D[grid3D].cap << " 3D grid: " << grid3D << endl;
    }*/
    /*while (numAdjust>0)
    {
        fscanf(fp, "%d %d %d %d %d %d %d\n", &x1, &y1, &l1, &x2, &y2, &l2, &reducedCap);
		reducedCap = reducedCap/2;

		k = l1-1;

		if (y1==y2)//horizontal edge
		{
			grid = y1*(xGrid-1)+x1+k*(xGrid-1)*yGrid;
			cap=h_edges3D[grid].cap;
			reduce=cap-reducedCap;
			h_edges3D[grid].cap=reducedCap;
			h_edges3D[grid].red=reduce;
			grid=y1*(xGrid-1)+x1;
			h_edges[grid].cap-=reduce;
			h_edges[grid].red += reduce;
			

		} else if (x1==x2)//vertical edge
		{
			grid = y1*xGrid+x1+k*xGrid*(yGrid-1);
			cap=v_edges3D[grid].cap;
			reduce=cap-reducedCap;
			v_edges3D[grid].cap=reducedCap;
			v_edges3D[grid].red=reduce;
			grid = y1*xGrid+x1;
			v_edges[grid].cap-=reduce;
			v_edges[grid].red += reduce;
			
		}
        numAdjust--;
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
    for (k=0; k<numLayers; k++)
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


void runFastRoute(parser::grGenerator grGen, string OutFileName, parser::CongestionMap& congestionMap, int maxMazeRound = 500)
{
//    char benchFile[FILESTRLEN];
    char routingFile[STRINGLEN];
    char degreeFile[STRINGLEN];
	char optionS[STRINGLEN];
    clock_t t1, t2, t3, t4;
    float gen_brk_Time, reading_Time, P1_Time, P2_Time, P3_Time, maze_Time, totalTime, congestionmap_time;
    int iter, last_totalOverflow, diff_totalOverflow, enlarge, ripup_threshold;
    int i, j,past_overflow,cur_overflow;
    int L_afterSTOP;
    int ESTEP1,CSTEP1, thStep1;
    int ESTEP2,CSTEP2, thStep2;
	int ESTEP3,CSTEP3, thStep3, tUsage, CSTEP4;
	int Ripvalue, LVIter, cost_step;
	int maxOverflow, past_cong, last_cong, finallength, numVia, ripupTH3D, newTH, healingTrigger;
	int updateType, minofl, minoflrnd, mazeRound, upType, cost_type, bmfl, bwcnt;
	Bool goingLV, healingNeed, noADJ, extremeNeeded, needOUTPUT;

    Bool input, WriteOut;
    input=WriteOut=0;
    numThreads = 1; // Just for test of lef/def and openDB
    cout << " nthreads: " << numThreads << endl;
    if(OutFileName != "")
    {
    	needOUTPUT = true;
    	strcpy(routingFile, OutFileName.c_str());
    }
	/*if(1)//strcmp(*argv,"./SPRoute")==0)
	{
		//argc--;  argv++;
		if(argc==0)
		{
			printf("--SPRoute --\n");
			printf("Usage: ./SPRoute  <input> -o <output>\n");
		}
	} else {
		printf("--SPRoute --\n");
		printf("Usage: ./SPRoute  <input> <output>\n");

		while(argc)
		{
			argc--; argv++;
		}
	}    
  

   if(argc != 1)
   {
      strcpy(benchFile, argv[1]);
      cout << benchFile << endl;
      input=1;
      
   } else {
	   printf("Usage: ./SPRoute  <input> -o <output> -t <nthreads> \n");
   }

   if(argc)
   {
   		
		strcpy(optionS, argv[2]);
		//argv++; argc--;
		cout << optionS << endl;
		if (strcmp(optionS,"-o")==0 )
		{
			if(argc)
		   {
		   	  
			  strcpy(routingFile, argv[3]);
			  //argv++; argc--;
			  cout << routingFile << endl;
			  
			  WriteOut=1;
			  needOUTPUT = TRUE;
		   } else {
			   printf("No output file specified\n");
			   exit(0);
		   }
		} else {
			printf("output option not recognized,  SPRoute will not generate output file\n");
			needOUTPUT = FALSE;
		}
   } else {
	   printf("No output file specified, SPRoute will not generate output file\n");
	   needOUTPUT = FALSE;
   }*/



	LB=0.9;
	UB=1.3;


	SLOPE=5;
	THRESH_M=20;
	ENLARGE=15;//5
	ESTEP1=10;//10
	ESTEP2=5;//5
	ESTEP3=5;//5
	CSTEP1=2;//5
	CSTEP2=2;//3
	CSTEP3=5;//15
	CSTEP4 = 1000;
	COSHEIGHT=4;
	L=0;
	VIA=2;
	L_afterSTOP=1;
	Ripvalue=-1;
	ripupTH3D = 10;
	goingLV = TRUE;
	noADJ = FALSE;
	thStep1 = 10;
	thStep2 = 4;
	healingNeed = FALSE;
	updateType = 0;
	LVIter = 3;
	extremeNeeded = FALSE;
	mazeRound = maxMazeRound;
	bmfl = BIG_INT;
	minofl = BIG_INT;

     //galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE> thread_local_storage;
    galois::setActiveThreads(numThreads);
    galois::on_each( 
            [&] (const unsigned tid, const unsigned numT)
            {
                printf("threadid: %d %d\n", tid, numT);
            }
            );

   cout << " nthreads: " << numThreads << endl;

    
   int finegrain = false;
   int thread_choice = 0;
   int thread_steps[6] = {28,14,8,4,1};
   int thread_livelock_limit[6] = {1,1,1,1,1};
   bool extrarun = false;
   int thread_livelock = 0;

   input = 1;
    if(input==1)
	{
		t1 = clock();
		printf("\nReading %s ...\n", benchFile);
		//readFile(benchFile);
		readGR(grGen);

		printf("\nReading Lookup Table ...\n");
		readLUT();
		printf("\nDone reading table\n\n");  

		


		t2 = clock();
		reading_Time = (float)(t2-t1)/CLOCKS_PER_SEC;
		printf("Reading Time: %f sec\n", reading_Time);
	    
		// call FLUTE to generate RSMT and break the nets into segments (2-pin nets)

		VIA=2;
		//viacost = VIA;
		viacost = 0;
		gen_brk_RSMT(FALSE, FALSE, FALSE, FALSE, noADJ);
		printf("first L\n");
		routeLAll(TRUE);
        gen_brk_RSMT(FALSE, TRUE,TRUE,FALSE, noADJ);
		//gen_brk_RSMT(TRUE, TRUE,TRUE,FALSE, noADJ); // for the contest only
		getOverflow2D( &maxOverflow);
		printf("second L\n");
		newrouteLAll(FALSE, TRUE);
		getOverflow2D( &maxOverflow); 
		spiralRouteAll ();
		newrouteZAll(10) ;
		printf("first Z\n");
		past_cong = getOverflow2D( &maxOverflow); 

		convertToMazeroute();

		enlarge = 10;
		newTH = 10;
		healingTrigger = 0;
		stopDEC = 0;
		upType  = 1;

		//iniBDE();

		costheight=COSHEIGHT;

		if (maxOverflow > 700) {
			costheight = 8;
			LOGIS_COF = 1.33;
			VIA = 0;
			THRESH_M = 0;
			CSTEP1 = 30;
			slope = BIG_INT;
		}


		for (i = 0; i < LVIter; i++) {

			LOGIS_COF = max (2.0/(1+log(maxOverflow)), LOGIS_COF);
			LOGIS_COF = 2.0/(1+log(maxOverflow));
			printf("LV routing round %d, enlarge %d \n", i,enlarge);
			routeLVAll(newTH, enlarge);

			past_cong = getOverflow2Dmaze( &maxOverflow , & tUsage); 

			enlarge += 5;
			newTH -= 5;
			if (newTH < 1) {
				newTH = 1;
			}
		}  
		
	//	past_cong = getOverflow2Dmaze( &maxOverflow); 
	
		t3 = clock();
		reading_Time = (float)(t3-t2)/CLOCKS_PER_SEC;
		printf("LV Time: %f sec\n", reading_Time);
		InitEstUsage();

		i=1;
		costheight=COSHEIGHT;
		enlarge=ENLARGE;
		ripup_threshold=Ripvalue;
		
		minofl = totalOverflow;
		stopDEC = FALSE;

		slope = 20;
		L = 1;
		cost_type = 1;

		InitLastUsage(upType);

		galois::InsertBag<int> net_shuffle[40]; //6*6
		OrderNetEdge* netEO = (OrderNetEdge*)calloc(2000, sizeof(OrderNetEdge));
		/*for(int netID = 0; netID < numValidNets; netID++)
		{
			int deg = sttrees[netID].deg;

	        netedgeOrderDec(netID, netEO);

	        TreeEdge* treeedges = sttrees[netID].edges;
	        TreeNode* treenodes = sttrees[netID].nodes;
	        // loop for all the tree edges (2*deg-3)
	        //int num_edges = 2*deg-3;

	        int x_sum = 0, y_sum = 0;

	        for(int nodeID=0; nodeID<deg; nodeID++)
	        {
	            x_sum += treenodes[nodeID].x;
	            y_sum += treenodes[nodeID].y;
	        } 
	        int x_mean = x_sum / 6;
	        int y_mean = y_sum / 6;

	        int x_block = x_mean / (xGrid / 6);
            if(x_block > 6) x_block = 6;
            int y_block = y_mean / (yGrid / 6);
            if(y_block > 6) y_block = 6;

	        net_shuffle[x_block * 6 + y_block].push(netID);

		}*/
		PRINT_HEAT = 0;
		//checkUsageCorrectness();
		galois::StatTimer roundtimer("round");
		unsigned long oldtime = 0;
		round_avg_dist = 0;
		round_avg_length = 0;
		while(totalOverflow>0)
		{

			if(THRESH_M>15) {
				THRESH_M-=thStep1;
			} else if(THRESH_M>=2) {
				THRESH_M-=thStep2;
			} else {
				THRESH_M = 0;
			}
			if(THRESH_M<=0) {
				THRESH_M=0;
			}
			//std::cout << "totalOverflow : " << totalOverflow << " enlarge: " << enlarge << std::endl; 
			if(totalOverflow>2000)
			{
				enlarge+=ESTEP1;//ENLARGE+(i-1)*ESTEP;
				cost_step = CSTEP1;
				updateCongestionHistory( i, upType);
				
			}
			else if(totalOverflow<500)
			{
				 
				cost_step = CSTEP3;
				enlarge+=ESTEP3;
				ripup_threshold = -1;
				updateCongestionHistory( i, upType);
			}  else	{
				cost_step = CSTEP2;
				enlarge+=ESTEP2;
				updateCongestionHistory( i, upType);
			}

			if(totalOverflow>15000 && maxOverflow > 400) {
				enlarge = max(xGrid,yGrid) / 30;  //This is the key!!!! to enlarge routing area!!!!
				//enlarge = max(xGrid,yGrid) / 10;
				slope = BIG_INT;
				//slope = 20;
				if (i == 5) {
					VIA = 0;
					LOGIS_COF = 1.33;
					ripup_threshold = -1;
				//	cost_type = 3;
					
				} else if (i > 6) {
					if (i %2 == 0) {
						LOGIS_COF += 0.5;
					}
					if (i > 20){
						break;
					}
				} 
				if (i > 10) {
					cost_type = 1;
					ripup_threshold = 0;
				}
			}

			 
			enlarge = min (enlarge, xGrid/2);
			//std::cout << "costheight : " << costheight << " enlarge: " << enlarge << std::endl; 
			costheight+=cost_step;
			//std::cout << "costheight : " << costheight << " enlarge: " << enlarge << std::endl; 
			mazeedge_Threshold = THRESH_M;

			if (upType == 3) {
				LOGIS_COF = max (2.0/(1+log(maxOverflow+max_adj)),LOGIS_COF);
			} else {
				LOGIS_COF = max (2.0/(1+log(maxOverflow)),LOGIS_COF);
			}

			if (i == 8) {
				L = 0; 
				upType = 2;
				InitLastUsage(upType);
			} 

			 
			if (maxOverflow == 1) {
				//L = 0;
				ripup_threshold = -1;
				slope = 5;
			}
 
			if (maxOverflow > 300 && past_cong > 15000) {
				L = 0;
			}
			//checkUsageCorrectness();

			//getOverflow2Dmaze(&maxOverflow , & tUsage); 

			printf("iteration %d, enlarge %d, costheight %d, threshold %d via cost %d \nlog_coef %f, healingTrigger %d cost_step %d slope %d L %f cost_type %d updatetype %d OBIM delta %d\n",
				i,enlarge,costheight,mazeedge_Threshold, VIA,LOGIS_COF, healingTrigger, cost_step, slope, L ,cost_type, upType, max(OBIM_delta, (int)(costheight / (2*slope))));
			//L = 2; 
			roundtimer.start();
			//galois::runtime::profileVtune( [&] (void) {
                round_num = i;
				if(finegrain)
				{
					printf("finegrain\n");

					mazeRouteMSMD_finegrain_spinlock(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
				}
				else
				{
					mazeRouteMSMD(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
				}
				/*if(finegrain == 0)
				{
					
 					mazeRouteMSMD(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
					
				}
				else if(finegrain >= 1 && finegrain <= 4)
				{
					cout << "concurrentNets:" << concurrentNets[finegrain] << endl;
					mazeRouteMSMD_finegrain_concurrent(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, concurrentNets[finegrain]);
				}
				else if(finegrain == 5)
				{
					printf("finegrain\n"); 
					mazeRouteMSMD_finegrain_spinlock(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
				}
				else {
					cout<<"unkown finegraph parameter: "<< finegrain <<  endl;
				}*/
			//}, "mazeroute");
			roundtimer.stop();
			cout << "round : " << i << " time(ms): " << roundtimer.get() - oldtime << " acc time(ms): " << roundtimer.get() << endl;
			oldtime = roundtimer.get();
			//checkUsageCorrectness();
            last_cong = past_cong;
 
			past_cong = getOverflow2Dmaze(&maxOverflow , & tUsage); 
			//if(i == 1)
			//	break;
            int nthreads_tmp = numThreads;
			if(past_cong > last_cong && !extrarun)  // Michael
			{ 
				if(!finegrain && nthreads_tmp != 1)
				{
					thread_livelock++;
					if(thread_livelock == 1)
					{
						thread_choice++;
						thread_livelock = 0;
                        if(nthreads_tmp < 6) {
						    galois::setActiveThreads(nthreads_tmp);
                            finegrain = true;
                        }
                        else
							galois::setActiveThreads(nthreads_tmp / 2);
					}
					cout << "nthreads :" <<	nthreads_tmp << " live lock cnt: " << thread_livelock << endl;
				}
			}
			extrarun = false;


			if (minofl > past_cong) {
				minofl =  past_cong;
				minoflrnd = i;
			}

			if (i == 8) {
				L = 1;
			}

			i++;

			if (past_cong < 200 && i > 30 && upType == 2 && max_adj <=20) {
				upType = 4;
				stopDEC = TRUE;
			}
			

			if (maxOverflow < 150) {
				if (i == 20 && past_cong > 200) {
					printf("Extra Run for hard benchmark\n");
					L = 0;
					upType = 3;
					stopDEC = TRUE;
					slope = 5;
					//galois::runtime::profileVtune( [&] (void) {
						if(finegrain)
						{
							printf("finegrain\n");

							mazeRouteMSMD_finegrain_spinlock(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
						}
						else
						{
							mazeRouteMSMD(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
						}
					//}, "mazeroute");
					last_cong = past_cong;
					past_cong = getOverflow2Dmaze(&maxOverflow , & tUsage);
					extrarun = true;

					str_accu(12);
					L = 1;
					stopDEC = FALSE;
					slope = 3;
					upType = 2;
				}
				if ( i == 35 && tUsage > 800000) {
					str_accu(25);
					extrarun = true;
				}
				if ( i == 50 && tUsage > 800000) {
					str_accu(40);
					extrarun = true;
				}

			}
			
			if (i > 50 ) {
				upType = 4;
				if (i > 70) {
					stopDEC = TRUE;
				}
			}
				
			if (past_cong > 0.7 * last_cong) {
				costheight += CSTEP3;
			}


			if (past_cong >= last_cong ) {
				VIA = 0; //is this good?
				healingTrigger ++;
			} 
			
			if (past_cong < bmfl) {
				bwcnt = 0;			
				if ( i > 140 || (i> 80 && past_cong < 20))
				{	
					copyRS();
					bmfl = past_cong;

					L = 0;
					slope = BIG_INT;
					//SLOPE = BIG_INT;
					//galois::runtime::profileVtune( [&] (void) {
						if(finegrain)
						{
							printf("finegrain\n");

							mazeRouteMSMD_finegrain_spinlock(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
						}
						else
						{
							mazeRouteMSMD(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type, net_shuffle);
						}
					//}, "mazeroute");
					last_cong = past_cong;
					past_cong = getOverflow2Dmaze(&maxOverflow , & tUsage);
					extrarun = true;
					if (past_cong < last_cong) {
						copyRS();
						bmfl = past_cong;
					}
					L = 1;
					slope = 5;
					//SLOPE = 5;
					if (minofl > past_cong) {
						minofl =  past_cong;
						minoflrnd = i;
					}
					if(bmfl < 72)
						break;

				}
			} else {
				bwcnt ++;
			}

			if (bmfl > 10) {
				if (bmfl > 30 && bmfl < 72 && bwcnt > 50) {
					break;
				}
				if (bmfl < 30 && bwcnt > 50) {
					break;
				}
				if (i >= mazeRound) {
					getOverflow2Dmaze( &maxOverflow, & tUsage);
					break;
				}
			}


			if (i >= mazeRound) {
				getOverflow2Dmaze( &maxOverflow, & tUsage);
				break;
			}
		}
		
		if (minofl > 0) {
			printf("\n\n minimal ofl %d, occuring at round %d\n\n",minofl,minoflrnd);
			copyBR();
		} 
			
		freeRR();
		

		checkUsage();

		printf("maze routing finished\n");
         
		t4 = clock();
		maze_Time = (float)(t4-t3)/CLOCKS_PER_SEC;
		//printf("P3 runtime: %f sec\n", maze_Time);

		printf("Final 2D results: \n");
		getOverflow2Dmaze( &maxOverflow , & tUsage);

		printf("\nLayer Assignment Begins");
		newLA ();
		printf("layer assignment finished\n");

		t2 = clock();
		gen_brk_Time = (float)(t2-t1)/CLOCKS_PER_SEC;
		//printf("2D + Layer Assignment Runtime: %f sec\n", gen_brk_Time); 

		costheight = 3;
		viacost = 1;

		if (gen_brk_Time < 60) {
			ripupTH3D = 15;
		} else if (gen_brk_Time < 120) {
			ripupTH3D = 18;
		} else {
			ripupTH3D = 20;
		}

		

		if (goingLV && past_cong == 0) {
			printf("Post Processing Begins \n");
			//mazeRouteMSMDOrder3D(enlarge, 0, ripupTH3D );
			
		//	mazeRouteMSMDOrder3D(enlarge, 0, 10 );
			if (gen_brk_Time > 120) {
				//mazeRouteMSMDOrder3D(enlarge, 0, 12 );
			}
			printf("Post Processsing finished, starting via filling\n");			

		}
		
		fillVIA();
		finallength = getOverflow3D(congestionMap);
		numVia= threeDVIA ();
		checkRoute3D();
		if (needOUTPUT) {
			writeRoute3D(routingFile, grGen);
		}
		
	}//Input ==1



	t4 = clock();
	maze_Time = (float)(t4-t1)/CLOCKS_PER_SEC;
	printf("Final routing length : %d\n",finallength);
	printf("Final number of via  : %d\n",numVia);
	printf("Final total length 1 : %d\n\n",finallength+numVia);
    	
    //printf("Final total length 3 : %d\n",(finallength+3*numVia));
	//printf("3D runtime: %f sec\n", maze_Time);

	//freeAllMemory();
	return ;
}



#endif