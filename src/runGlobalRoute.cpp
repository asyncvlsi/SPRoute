#include "sproute.h"
#include "global.h"
#include "soft_cap.h"

namespace sproute {

void SPRoute::ReadGR(sproute_db::grGenerator& grGen, Algo algo)
{
    FILE *fp;
    int pinX, pinY, pinL, netID, numPins, pinInd, grid, newnetID, invalid_netID, segcount, minwidth;
    float pinX_in,pinY_in;
    int maxDeg = 0;
    int pinXarray[MAXNETDEG], pinYarray[MAXNETDEG], pinLarray[MAXNETDEG];
    string netName;
    bool remove;
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

	if(verbose_ > none) {
		printf("\n");
		printf("grid %d %d %d\n", xGrid, yGrid, numLayers);
		for (int i=0;i<numLayers;i++)
		{ 
			printf("Layer %d vertical capacity %d, horizontal capacity %d\n", i, vCapacity3D[i], hCapacity3D[i]);
		}
	
		printf("total vertical capacity %d\n", vCapacity);
		printf("total horizontal capacity %d\n", hCapacity);
		printf("num net %d\n", numNets);
	}

    // allocate memory for nets
    nets = (Net**) malloc(numNets*sizeof(Net*));
    invalid_nets = (Net**) malloc(numNets*sizeof(Net*));
    for(int i=0; i<numNets; i++)
    {
        nets[i] = new Net;
        invalid_nets[i] = new Net;
    }
    seglistIndex = (int*) malloc(numNets*sizeof(int));
    
    // read nets information from the input file
    segcount = 0;
    newnetID = 0;
    invalid_netID = 0;


    for(int i=0; i<numNets; i++)
    {
        net++;
        netName = grGen.grnets.at(i).name;
        netID = grGen.grnets.at(i).idx;
        numPins = grGen.grnets.at(i).numPins;
        if(verbose_ > none && numPins > 1000)
            cout << "reading large net: " << netName << " " << numPins << endl;
        if(verbose_ > 2)
	    cout << netName << " " << netID << " " << numPins << endl;
        if (1)
        {
            pinInd = 0;
            for(int j=0; j<numPins; j++) {
                //fscanf(fp, "%f	%f	%d\n", &pinX_in, &pinY_in, &pinL);
                pinX_in = grGen.grnets.at(i).pins.at(j).x;
                pinY_in = grGen.grnets.at(i).pins.at(j).y;
                pinL = grGen.grnets.at(i).pins.at(j).z;

                pinX = sproute_db::find_Gcell(pinX_in, defDB.xGcellBoundaries);
                pinY = sproute_db::find_Gcell(pinY_in, defDB.yGcellBoundaries);
            
                if(!(pinX < 0 || pinX > xGrid || pinY < -1 || pinY > yGrid|| pinL > numLayers || pinL<=0))
                {
                    
                    remove = false;
                    for(int k=0; k<pinInd; k++)
                    {
                        if(pinX==pinXarray[k] && pinY==pinYarray[k] && pinL==pinLarray[k])
                        {remove=true; break;}
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
		nets[newnetID]->name = netName;
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
                invalid_nets[invalid_netID]->name = netName;
                invalid_nets[invalid_netID]->netIDorg = netID;
                invalid_nets[invalid_netID]->numPins = numPins;
                invalid_nets[invalid_netID]->deg = 1;
                invalid_nets[invalid_netID]->pinX = (short*) malloc(sizeof(short));
                invalid_nets[invalid_netID]->pinY = (short*) malloc(sizeof(short));
                invalid_nets[invalid_netID]->pinL = (short*) malloc(sizeof(short));

		
                invalid_nets[invalid_netID]->pinX[0] = pinXarray[0];
                invalid_nets[invalid_netID]->pinY[0] = pinYarray[0];
                invalid_nets[invalid_netID]->pinL[0] = pinLarray[0];
                
                //cout << "netName: " << netName << " pinInd: " << pinInd << endl;
                //cout << " X Y Z: " << pinXarray[0] << " " << pinYarray[0] << " " << pinLarray[0] << endl;
                invalid_netID++;
            }
        }//if

        if(verbose_ > none && numPins > 1000)
            cout << "reading large net finished: " << netName << endl;
        if(verbose_ > none && i%100000 == 0)
            cout << "read " << i << " nets." << endl;
    } // loop i
	if(verbose_ > none)
    	printf("the total net number is %d\n\n",net);

    if((pinInd>1))
    {
		seglistIndex[newnetID] = segcount; // the end pointer of the seglist
    }
    numValidNets = newnetID;
    numInvalidNets = invalid_netID;
    
       // allocate memory and initialize for edges
	float* rudy = new float [xGrid * yGrid];
	
	max_rudy = ComputeRudy(rudy, algo);
	
	if(algo == DRC_MAP) {
		PlotDrcMap();
		exit(0);
	}
	float avg_pin_den = ComputePinDensity(rudy, algo);
	if(verbose_ > none)
		cout << "avg pin density: " << avg_pin_den << endl;
	if(algo == RUDY || algo == PIN_DENSITY || algo == DRC_MAP) {
		exit(0) ;
	}

	float rudy_weight = (max_rudy < 48)? DEFAULT_RUDY_WEIGHT : 0.01;
	float pin_density_weight = (max_rudy < 48)? DEFAULT_PIN_DENSITY_WEIGHT : 1.00;

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
	InvalidNetsAdj(invalid_nets, xGrid, yGrid);
	MemPinAdj(xGrid, yGrid);


	treeOrderCong = NULL;
	stopDEC = false;

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

	if(verbose_ > none) {
		printf("# valid nets: %d\n", numValidNets);
		printf("# segments: %d\n", segcount);
		printf("maxDeg:     %d\n", maxDeg);
		printf("\nDone getting input\n");
		printf("MD: %d, AD: %.2f, #nets: %d, #routed nets: %d\n", MD, (float)TD/newnetID, numNets, newnetID); 
		printf("TC is %d\n",TC);  
	}
	sttreesBK = NULL;
}

void SPRoute::RunGlobalRoute(string OutFileName, int maxMazeRound, Algo algo) {

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
	int updateType, minofl, minoflrnd = 0, mazeRound, upType, cost_type, bmfl, bwcnt;
	bool goingLV, healingNeed, noADJ, extremeNeeded, needOUTPUT;

	int parts = 128;
	int max_nets_per_part = 64;
	galois::LargeArray<bool> done;
	float astar_weight = 1.0;
    done.allocateBlocked(numValidNets + 1);
    for (int i = 0; i < numValidNets; i++) {
      done[i] = false;
	}

    bool input, WriteOut;
    input=WriteOut=0;
	if(verbose_ > none) {
    	cout << " nthreads: " << numThreads << endl;
	}
    if(OutFileName != "")
    {
    	needOUTPUT = true;
    }

	LB=0.9;
	UB=1.3;


	SLOPE=5;
	THRESH_M=20;
	ENLARGE=115;//5
	ESTEP1=30;//10
	ESTEP2=30;//5
	ESTEP3=30;//5
	CSTEP1=2;//5
	CSTEP2=2;//3
	CSTEP3=5;//15
	CSTEP4 = 1000;
	COSHEIGHT=40;
	L=0;
	VIA=2;
	L_afterSTOP=1;
	Ripvalue=-1;
	ripupTH3D = 10;
	goingLV = true;
	noADJ = false;
	thStep1 = 10;
	thStep2 = 4;
	healingNeed = false;
	updateType = 0;
	LVIter = 3;
	extremeNeeded = false;
	mazeRound = maxMazeRound;
	bmfl = BIG_INT;
	minofl = BIG_INT;

     //galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE> thread_local_storage;
    galois::setActiveThreads(numThreads);
	if(verbose_ > none) {
    	galois::on_each( 
            [&] (const unsigned tid, const unsigned numT)
            {
                printf("threadid: %d total numT: %d\n", tid, numT);
            }
            );
	}
    
   int thread_choice = 0;
   int thread_steps[6] = {28,14,8,4,1};
   int thread_livelock_limit[6] = {1,1,1,1,1};
   bool extrarun = false;
   int thread_livelock = 0;

    if(1)
	{
		t1 = clock();
		if(verbose_ > none) {
			printf("\nReading %s ...\n", benchFile);
		}
		//readFile(benchFile);
		ReadGR(grGen, algo);

		if(verbose_ > none)
			printf("\nReading Lookup Table ...\n");
        if (getenv ("ACT_HOME") == NULL) {
  		   	readLUT();
        }
        else {
	    	char *lpath = (char *)malloc (strlen (getenv ("ACT_HOME")) + 6);
			sprintf (lpath, "%s/lib/", getenv ("ACT_HOME"));
			readLUT (lpath);
            free (lpath);
        }
		if(verbose_ > none)
			printf("\nDone reading table\n\n");  

		


		t2 = clock();
		reading_Time = (float)(t2-t1)/CLOCKS_PER_SEC;
		if(verbose_ > none)
			printf("Reading Time: %f sec\n", reading_Time);
	    
		// call FLUTE to generate RSMT and break the nets into segments (2-pin nets)

		VIA=2;
		//viacost = VIA;
		viacost = 0;
		gen_brk_RSMT(false, false, false, false, noADJ);
		if(verbose_ > none) {
			printf("first L\n");
		}
		routeLAll(true);
        gen_brk_RSMT(false, true,true,false, noADJ);
		//gen_brk_RSMT(true, true,true,false, noADJ); // for the contest only
		getOverflow2D( &maxOverflow);
		if(verbose_ > none) {
			printf("second L\n");
		}
		newrouteLAll(false, true);
		getOverflow2D( &maxOverflow); 
		spiralRouteAll ();
		newrouteZAll(10) ;
		if(verbose_ > none) {
			printf("first Z\n");
		}
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
			VIA = 1;
			THRESH_M = 0;
			CSTEP1 = 30;
			slope = BIG_INT;
		}


		for (i = 0; i < LVIter; i++) {

			LOGIS_COF = max (2.0/(1+log(maxOverflow)), LOGIS_COF);
			LOGIS_COF = 2.0/(1+log(maxOverflow));
			if(verbose_ > none) {
				printf("LV routing round %d, enlarge %d \n", i,enlarge);
			}
			routeLVAll(newTH, enlarge);

			past_cong = getOverflow2Dmaze( &maxOverflow , & tUsage, false); 

			enlarge += 5;
			newTH -= 5;
			if (newTH < 1) {
				newTH = 1;
			}
		}  
	
		t3 = clock();
		reading_Time = (float)(t3-t2)/CLOCKS_PER_SEC;
		if(verbose_ > none) {
			printf("LV Time: %f sec\n", reading_Time);
		}
		InitEstUsage();

		i=1;
		costheight=COSHEIGHT;
		enlarge=ENLARGE;
		ripup_threshold=Ripvalue;
		
		minofl = past_cong;
		stopDEC = false;

		slope = 20;
		L = 1;
		cost_type = 1;

		InitLastUsage(upType);

		//checkUsageCorrectness();
		galois::StatTimer roundtimer("round");
		galois::StatTimer rudytimer("rudy");
		unsigned long oldtime = 0;
		round_avg_dist = 0;
		round_avg_length = 0;

		galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>
      		thread_local_storage{};

		while(totalOverflow>0)
		{

			THRESH_M = 0;
			//std::cout << "totalOverflow : " << totalOverflow << " enlarge: " << enlarge << std::endl; 
			if(totalOverflow>2000)
			{
				enlarge+=ESTEP1;//ENLARGE+(i-1)*ESTEP;
				cost_step = CSTEP1;
				updateCongestionHistory(upType);
				
			}
			else if(totalOverflow<500)
			{
				 
				cost_step = CSTEP3;
				enlarge+=ESTEP3;
				ripup_threshold = -1;
				updateCongestionHistory(upType);
			}  else	{
				cost_step = CSTEP2;
				enlarge+=ESTEP2;
				updateCongestionHistory(upType);
			}

			if(totalOverflow>15000 && maxOverflow > 400) {
				//enlarge = max(xGrid,yGrid) / 30;  //This is the key!!!! to enlarge routing area!!!!
				//enlarge = max(xGrid,yGrid) / 10;
				slope = BIG_INT;
				//slope = 20;
				if (i == 5) {
					VIA = 1;
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

			 
			enlarge = min (enlarge, max(xGrid, yGrid)/2);
			costheight+=cost_step;
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
			if(verbose_ > none) {
				printf("iteration %d, enlarge %d, costheight %d, threshold %d via cost %d \nlog_coef %f, healingTrigger %d cost_step %d slope %d L %f cost_type %d updatetype %d OBIM delta %d\n",
					i,enlarge,costheight,mazeedge_Threshold, VIA,LOGIS_COF, healingTrigger, cost_step, slope, L ,cost_type, upType, max(OBIM_delta, (int)(costheight / (2*slope))));
			}
			//L = 2; 
			roundtimer.start();
			//galois::runtime::profileVtune( [&] (void) {
				switch(algo) {
					case FineGrain: {
						printf("finegrain\n");
						//mazeRouteMSMD_finegrain_spinlock(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type);
						break;
					}
					case NonDet: {
						if(i == 1)
							UndoneFilter(done);
						mazeRouteMSMD(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, false /*!(i % 3)*/, cost_type, done, thread_local_storage);
						break;
					}
					case Det: {
						if(i == 1) { //first iteration
							UndoneFilter(done, true);
							if(verbose_ > none)
								cout << "num of small undone nets: " << n_small_undone <<  endl;
							
							int nets_per_part;

							if(n_small_undone > parts * max_nets_per_part)
								nets_per_part = max_nets_per_part;
							else
								nets_per_part = max(8, n_small_undone / parts + 1);

							int counted_num_part = n_small_undone / nets_per_part + 1;
							if(verbose_ > none)
								cout << "acc_count: " << acc_count << "avg nets_per_part: "  << nets_per_part << " counted parts: " << counted_num_part << endl;
							std::vector<int> part;
							
							std::vector<std::vector<int>> vecParts;
							//vecParts.resize(counted_num_part)
							
							rudytimer.start();
							vecParts.resize(counted_num_part);
							for (int netID = 0; netID < numValidNets; netID ++) {
								if(nets[netID]->small && !done[netID]) {
									vecParts[netID % counted_num_part].push_back(netID);
								}
							}
							/*for(int batchID = 0; batchID < vecParts.size(); batchID++) {
								cout << vecParts[batchID].size() << " ";
							}
							cout << endl;*/
							rudytimer.stop();
							
							if(part.size() != 0)
								vecParts.push_back(part);
							//std::cout << "parts: " << vecParts.size() << std::endl;
						
							mazeRouteMSMDDetPart_Astar_Local(i, enlarge, costheight, ripup_threshold,
												mazeedge_Threshold, false, cost_type, vecParts.size(),
												vecParts, done, thread_local_storage);
						}
						else { // 2nd iteration and after
							if(max_rudy > 50 && i == 2)
								max_nets_per_part = 128;

							if(max_nets_per_part >= 16)
								max_nets_per_part /= 2;
							UndoneFilter(done);
							int nets_per_part;

							nets_per_part = max_nets_per_part;
							int counted_num_part = (numValidNets - acc_count) / nets_per_part + 1;
							if(verbose_ > none)
								cout << "avg nets_per_part: "  << nets_per_part << " counted parts: " << counted_num_part << endl;
							std::vector<int> part;
							
							std::vector<std::vector<int>> vecParts;
						
							rudytimer.start();
							vecParts.resize(counted_num_part);
							int undone_cnt = numValidNets;
							if(i == 2)
								undone_cnt = UndoneNetOrderX(done);
							else if(i == 3)
								undone_cnt = UndoneNetOrderY(done);
						

							for (int netOrderID = 0; netOrderID < undone_cnt; netOrderID++) {
								int netID;
								if(undone_cnt == numValidNets)
									netID = netOrderID;
								else
									netID = treeOrderCong[netOrderID].treeIndex;

								if(undone_cnt != numValidNets && netOrderID < 10 && verbose_ > none) 
									cout << netOrderID << " " << netID << " " << treeOrderCong[netOrderID].xmin << endl;

								if(!done[netID]) {
									if(max_nets_per_part <= 16) {
										int original_batch_id;
										if(i != 1)
											original_batch_id = (netOrderID + i) % counted_num_part;
										else
											original_batch_id = netOrderID / counted_num_part;
										bool pushed = false;
										for(int batchID = original_batch_id; batchID < original_batch_id + counted_num_part; batchID++) {
											if(vecParts[batchID % counted_num_part].size() < nets_per_part) {
												vecParts[batchID % counted_num_part].push_back(netID);
												pushed = true;
												break;
											}
										}
										if(!pushed) {
											cout << "unpushed: " << netID << " " << counted_num_part << " " << original_batch_id << endl;
											for(int batchID = 0; batchID < vecParts.size(); batchID++) {
												cout << vecParts[batchID].size() << " ";
											}
											exit(1);
										}
									}
									else
										vecParts[(netOrderID + i) % counted_num_part].push_back(netID);
								}
							}
							for(int batchID = 0; batchID < vecParts.size(); batchID++) {
								cout << vecParts[batchID].size() << " ";
							}
							if(verbose_ > none)
								cout << endl;
							rudytimer.stop();
							
							if(part.size() != 0)
								vecParts.push_back(part);
							if(verbose_ > none)
								std::cout << "parts: " << vecParts.size() << std::endl;
						
							mazeRouteMSMDDetPart_Astar_Local(i, enlarge, costheight, ripup_threshold,
												mazeedge_Threshold, false, cost_type, vecParts.size(),
												vecParts, done, thread_local_storage);
						}
						break;
					}
					
					default: {
						cout << " unkown algo: " << algo << endl;
						exit(1);
					}
				}
				
			//}, "mazeroute");
			roundtimer.stop();
			if(verbose_ > none)
				cout << "round : " << i << " time(ms): " << roundtimer.get() - oldtime << " acc time(ms): " << roundtimer.get() << " scheduling time: " << rudytimer.get()<< endl;
			oldtime = roundtimer.get();
			//checkUsageCorrectness();
            last_cong = past_cong;
 
			past_cong = getOverflow2Dmaze(&maxOverflow , & tUsage); 
			//if(i == 1)
			//	break;
            int nthreads_tmp = numThreads;
			if(algo == NonDet && past_cong > last_cong && !extrarun)  // Michael
			{ 
				if(algo != FineGrain && nthreads_tmp != 1)
				{
					thread_livelock++;
					if(thread_livelock == 1)
					{
						thread_choice++;
						thread_livelock = 0;
                        if(nthreads_tmp < 6) {
						    galois::setActiveThreads(2);
                            numThreads = 2;
                            //algo = FineGrain;
                        }
                        else {
                            numThreads = numThreads / 2;
							galois::setActiveThreads(numThreads);
					    }
                    }
				}
			}

			if(verbose_ > none)
				cout << "nthreads :" <<	numThreads << endl;
			extrarun = false;

			if (minofl > past_cong) {
				minofl =  past_cong;
				minoflrnd = i;
				if(i > 5)
					copyRS();
			}
			//if(i == 2) break;
			if(i == 4 && minofl > 5000)
				break; 

			if (i == 8) {
				L = 1;
			}

			i++;

			if (past_cong < 200 && i > 30 && upType == 2 && max_adj <=20) {
				upType = 4;
				stopDEC = true;
			}
			
			if (i > 50 ) {
				upType = 4;
				if (i > 70) {
					stopDEC = true;
				}
			}
				
			if (past_cong > 0.7 * last_cong) {
				costheight += CSTEP3;
			}


			if (past_cong >= last_cong ) {
				//VIA = 0; //is this good?
				healingTrigger ++;
			} 
			
			if (past_cong < bmfl) {
				bwcnt = 0;			
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

		if(verbose_ > none)
		printf("maze routing finished\n");
         
		t4 = clock();
		maze_Time = (float)(t4-t3)/CLOCKS_PER_SEC;
		//printf("P3 runtime: %f sec\n", maze_Time);
		if(verbose_ > none)
			printf("Final 2D results: \n");
		getOverflow2Dmaze( &maxOverflow , & tUsage);

		if(verbose_ > none)
			printf("\nLayer Assignment Begins");

		newLA();
		if(verbose_ > none)
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
			if(verbose_ > none)
				printf("Post Processing Begins \n");
			//mazeRouteMSMDOrder3D(enlarge, 0, ripupTH3D );
			
			//	mazeRouteMSMDOrder3D(enlarge, 0, 10 );
			if (gen_brk_Time > 120) {
				//mazeRouteMSMDOrder3D(enlarge, 0, 12 );
			}
			if(verbose_ > none)
				printf("Post Processsing finished, starting via filling\n");			

		}
		
		fillVIA();
		finallength = getOverflow3D();
		numVia= threeDVIA ();
		checkRoute3D();
		//RCEstimate();
		//printUndone(done);
		WriteGuideToPhydb();
		/*if (needOUTPUT) {
			string overflowFile = "overflow.guide";
			writeOverflow(overflowFile, grGen);
		}*/
		
	}//Input ==1



	t4 = clock();
	maze_Time = (float)(t4-t1)/CLOCKS_PER_SEC;
	if(verbose_ > none) {
		printf("Final routing length : %d\n",finallength);
		printf("Final number of via  : %d\n",numVia);
		printf("Final total length 1 : %d\n\n",finallength+numVia);
	}
    	
    //printf("Final total length 3 : %d\n",(finallength+3*numVia));
	//printf("3D runtime: %f sec\n", maze_Time);

	//freeAllMemory();
	return ;
}

void SPRoute::WriteRoutedGrid(FILE* fp, std::set<sproute_db::Point3D<int>>& routedGrid)
{
	for(auto p : routedGrid)
	{
		int llx  = defDB.xGcellBoundaries.at(p.x);		
		int lly = defDB.yGcellBoundaries.at(p.y);
		int urx = defDB.xGcellBoundaries.at(p.x + 1);
		int ury  = defDB.yGcellBoundaries.at(p.y + 1);
		int layer = p.z;

		if(layer * 2 < lefDB.layers.size())
		{
			string layerName = lefDB.layers.at(layer * 2).name;
			fprintf(fp, "%d %d %d %d %s\n", llx, lly, urx, ury, layerName.c_str());
		}
		else {
			cout << "Error in writing guide: exceeds top layer! " << endl;
			exit(1);
		}
	}
}

void SPRoute::WriteRoutedGridToPhydb(int netID, std::set<sproute_db::Point3D<int>>& routedGrid)
{
	auto design_p = db_ptr_->GetDesignPtr();
	for(auto p : routedGrid)
	{
		int llx  = defDB.xGcellBoundaries.at(p.x);		
		int lly = defDB.yGcellBoundaries.at(p.y);
		int urx = defDB.xGcellBoundaries.at(p.x + 1);
		int ury  = defDB.yGcellBoundaries.at(p.y + 1);
		int layer = p.z;

		if(layer * 2 < lefDB.layers.size())
		{
			string layerName = lefDB.layers.at(layer * 2).name;
			design_p->InsertRoutingGuide(netID, llx, lly, urx, ury, layer * 2);
			//fprintf(fp, "%d %d %d %d %s\n", llx, lly, urx, ury, layerName.c_str());
		}
		else {
			cout << "Error in writing guide: exceeds top layer! " << endl;
			exit(1);
		}
	}
}

void SPRoute::WriteGuideToFile(string guideFileName)
{
	short *gridsX, *gridsY, *gridsL;
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	using Point3D = sproute_db::Point3D<int>;
	
	fp=fopen(guideFileName.c_str(), "w");
    if (fp == NULL) {
        printf("Error in opening %s\n", guideFileName.c_str());
        exit(1);
    }

	for(netID=0;netID<numValidNets;netID++)
	{
		string netName(nets[netID]->name);
		bool print = false;
		fprintf(fp, "%s\n", nets[netID]->name.c_str());
		fprintf(fp, "(\n");
        treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		
		nodes = sttrees[netID].nodes;

		int grNetID = defDB.netName2netidx.find(nets[netID]->name)->second; 
		auto grNet = grGen.grnets.at(grNetID);

		std::set<Point3D> routedGrid;

		for(auto p : grNet.pinRegion)
		{
			int botL = p.z - 1;
			int topL = botL + 2;
			for (int layer = botL; layer <= topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				assert(p.x < xGrid && p.y < yGrid);
				if(layer < numLayers)
                	routedGrid.insert(Point3D(p.x, p.y, layer));
            }
		}

		for(int i = 0; i < deg; i++)
		{	
			if(nodes[i].botL == 0)
				nodes[i].topL = (nodes[i].topL >= 2)? nodes[i].topL : 2;

			for (int layer = nodes[i].botL; layer <= nodes[i].topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				assert(nodes[i].x < xGrid && nodes[i].y < yGrid);
				if(layer < numLayers)
					routedGrid.insert(Point3D(nodes[i].x, nodes[i].y, layer));
			}
		}

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			edge = sttrees[netID].edges[edgeID];
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {
				
				routeLen = treeedge->route.routelen;
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;
				gridsL = treeedge->route.gridsL;

				if(print)
				{
					cout << "routelen: " << routeLen << endl;
				}

				string direction;
				bool output = false;
				int start, end;
				for(int output_cnt = 0; output_cnt < 1; output_cnt++)
				{
					for (i = 0; i <= routeLen; i ++) {
						routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i]));
						if(i % 5 == 3) {
							if(gridsL[i] <= numLayers - 2) {
								routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i] + 1));
								//cout << "add top" << endl;
							}
							else if(gridsL[i] >= 2) {//metal5 adding a metal4 
								routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i] - 1));
							}
						}
					}
				}
			}
		}

		WriteRoutedGrid(fp, routedGrid);
		fprintf(fp, ")\n");
	}

	for(netID=0;netID<numInvalidNets;netID++)
	{
		fprintf(fp, "%s\n", invalid_nets[netID]->name.c_str());
		fprintf(fp, "(\n");

		string netName(invalid_nets[netID]->name);
		bool print = false;

		if(defDB.netName2netidx.count(netName) == 0)
		{
			cout << "Error: unable to find net: " << netName << endl;
			exit(1);
		}
		int grNetID = defDB.netName2netidx.find(netName)->second; 
		auto grNet = grGen.grnets.at(grNetID);

		std::set<Point3D> routedGrid;

		for(auto p : grNet.pinRegion)
		{
			int botL = p.z - 1;
			int topL = botL + 2;

			for (int layer = botL; layer <= topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				if(layer * 2 < lefDB.layers.size())
				{
					routedGrid.insert(Point3D(p.x, p.y, layer));
				}
			}
		}

		WriteRoutedGrid(fp, routedGrid);
		fprintf(fp, ")\n");
	}

	fclose(fp);
}


void SPRoute::WriteOverflow(string overflowFile)
{
	short *gridsX, *gridsY, *gridsL;
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	using Point3D = sproute_db::Point3D<int>;
	
	fp=fopen(overflowFile.c_str(), "w");
    if (fp == NULL) {
        printf("Error in opening %s\n", overflowFile.c_str());
        exit(1);
    }

	std::set<Point3D> overflowGrid;
	string overflow_name = "overflow";
	fprintf(fp, "%s\n", overflow_name.c_str());
	fprintf(fp, "(\n");
        
	for (int k = 0; k < numLayers; k++) {
		for (int i = 0; i < yGrid; i++) {
			for (int j = 0; j < xGrid - 1; j++) {
				int grid = i * (xGrid - 1) + j + k * (xGrid - 1) * yGrid;
				int overflow = h_edges3D[grid].usage - h_edges3D[grid].cap;

				if (overflow > 0) {
					overflowGrid.insert(Point3D(j, i, k));
				}
			}
		}
		for (int i = 0; i < yGrid - 1; i++) {
			for (int j = 0; j < xGrid; j++) {
				int grid = i * xGrid + j + k * xGrid * (yGrid - 1);
				
				int overflow = v_edges3D[grid].usage - v_edges3D[grid].cap;

				if (overflow > 0) {
					overflowGrid.insert(Point3D(j, i, k));
				}
			}
		}
	}

	WriteRoutedGrid(fp, overflowGrid);
	fprintf(fp, ")\n");

	fclose(fp);
}

void SPRoute::WriteGuideToPhydb() {
	short *gridsX, *gridsY, *gridsL;
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	using Point3D = sproute_db::Point3D<int>;
    
	for(netID=0;netID<numValidNets;netID++)
	{
		string netName(nets[netID]->name);
		bool print = false;
        treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		
		nodes = sttrees[netID].nodes;

		int grNetID = defDB.netName2netidx.find(nets[netID]->name)->second; 
		auto grNet = grGen.grnets.at(grNetID);

		std::set<Point3D> routedGrid;

		for(auto p : grNet.pinRegion)
		{
			int botL = p.z - 1;
			int topL = botL + 2;
            
			for (int layer = botL; layer <= topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				assert(p.x < xGrid && p.y < yGrid);
				if(layer < numLayers)
                	routedGrid.insert(Point3D(p.x, p.y, layer));
            }
		}

		for(int i = 0; i < deg; i++)
		{	
			if(nodes[i].botL == 0)
				nodes[i].topL = (nodes[i].topL >= 2)? nodes[i].topL : 2;

			for (int layer = nodes[i].botL; layer <= nodes[i].topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				assert(nodes[i].x < xGrid && nodes[i].y < yGrid);
				if(layer < numLayers)
					routedGrid.insert(Point3D(nodes[i].x, nodes[i].y, layer));
			}
		}

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			edge = sttrees[netID].edges[edgeID];
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {
				
				routeLen = treeedge->route.routelen;
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;
				gridsL = treeedge->route.gridsL;

				if(print)
				{
					cout << "routelen: " << routeLen << endl;
				}

				string direction;
				bool output = false;
				int start, end;
				for(int output_cnt = 0; output_cnt < 1; output_cnt++)
				{
					for (i = 0; i <= routeLen; i ++) {
						routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i]));
						if(i % 5 == 3) {
							if(gridsL[i] <= numLayers - 2) {
								routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i] + 1));
								//cout << "add top" << endl;
							}
							else if(gridsL[i] >= 2) {//metal5 adding a metal4 
								routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i] - 1));
							}
						}
					}
				}
			}
		}
		int lef_netID = defDB.netName2netidx[netName];
		WriteRoutedGridToPhydb(lef_netID, routedGrid);
	}

	for(netID=0;netID<numInvalidNets;netID++)
	{
		string netName(invalid_nets[netID]->name);
		bool print = false;

		if(defDB.netName2netidx.count(netName) == 0)
		{
			cout << "Error: unable to find net: " << netName << endl;
			exit(1);
		}
		int grNetID = defDB.netName2netidx.find(netName)->second; 
		auto grNet = grGen.grnets.at(grNetID);

		std::set<Point3D> routedGrid;

		for(auto p : grNet.pinRegion)
		{
			int botL = p.z - 1;
			int topL = botL + 2;

			for (int layer = botL; layer <= topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				if(layer * 2 < lefDB.layers.size())
				{
					routedGrid.insert(Point3D(p.x, p.y, layer));
				}
			}
		}
		int lef_netID = defDB.netName2netidx[netName];
		WriteRoutedGridToPhydb(lef_netID, routedGrid);
	}

}


void SPRoute::WriteGcellGridToPhydb() {
	auto design_p = db_ptr_->GetDesignPtr();

	for(auto gcellgrid : defDB.gcellGrids) {
		design_p->AddGcellGrid(phydb::StrToXYDirection(gcellgrid.direction), gcellgrid.start, gcellgrid.numBoundaries, gcellgrid.step);
	}
}


} //namespace sproute
