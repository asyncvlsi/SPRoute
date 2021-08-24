#include "sproute.h"
#include "utility.h"

namespace sproute {
void printEdge(int netID, int edgeID)
{
    int i;
    TreeEdge edge;
    TreeNode *nodes;
    
    edge = sttrees[netID].edges[edgeID];
    nodes = sttrees[netID].nodes;
    
    printf("edge %d: (%d, %d)->(%d, %d)\n", edgeID, nodes[edge.n1].x, nodes[edge.n1].y, nodes[edge.n2].x, nodes[edge.n2].y);
    for(i=0; i<=edge.route.routelen; i++)
    {
        printf("(%d, %d) ", edge.route.gridsX[i], edge.route.gridsY[i]);
    }
    printf("\n");
}

void plotTree(int netID)
{
	short *gridsX, *gridsY;
    int i, j, Zpoint, n1, n2, x1, x2, y1, y2, ymin, ymax, xmin, xmax;
    
    RouteType routetype;
    TreeEdge *treeedge;
    TreeNode *treenodes;
    FILE *fp;

    
    xmin = ymin = 1e5;
    xmax = ymax = 0;
    
    fp=fopen("plottree", "w");
    if (fp == NULL) {
        printf("Error in opening file plottree\n");
        exit(1);
    }

    treenodes = sttrees[netID].nodes;
    for(i=0; i<sttrees[netID].deg; i++)
    {
        x1 = treenodes[i].x;
        y1 = treenodes[i].y;
        fprintf(fp, "%f %f\n", (float)x1-0.1, (float)y1);
        fprintf(fp, "%f %f\n", (float)x1, (float)y1-0.1);
        fprintf(fp, "%f %f\n", (float)x1+0.1, (float)y1);
        fprintf(fp, "%f %f\n", (float)x1, (float)y1+0.1);
        fprintf(fp, "%f %f\n", (float)x1-0.1, (float)y1);
        fprintf(fp, "\n");
    }
    for(i=sttrees[netID].deg; i<sttrees[netID].deg*2-2; i++)
    {
        x1 = treenodes[i].x;
        y1 = treenodes[i].y;
        fprintf(fp, "%f %f\n", (float)x1-0.1, (float)y1+0.1);
        fprintf(fp, "%f %f\n", (float)x1+0.1, (float)y1-0.1);
        fprintf(fp, "\n");
        fprintf(fp, "%f %f\n", (float)x1+0.1, (float)y1+0.1);
        fprintf(fp, "%f %f\n", (float)x1-0.1, (float)y1-0.1);
        fprintf(fp, "\n");
    }
        
    for(i=0; i<sttrees[netID].deg*2-3; i++)
    {
if(1)//i!=14)
{
        treeedge = &(sttrees[netID].edges[i]);
        
        n1 = treeedge->n1;
        n2 = treeedge->n2;
        x1 = treenodes[n1].x;
        y1 = treenodes[n1].y;
        x2 = treenodes[n2].x;
        y2 = treenodes[n2].y;
        xmin = min(xmin, min(x1, x2));
        xmax = max(xmax, max(x1, x2));
        ymin = min(ymin, min(y1, y2));
        ymax = max(ymax, max(y1, y2));
                
        routetype = treeedge->route.type;
        
        if(routetype==LROUTE) // remove L routing
        {
            if(treeedge->route.xFirst)
            {
                fprintf(fp, "%d %d\n", x1, y1);
                fprintf(fp, "%d %d\n", x2, y1);
                fprintf(fp, "%d %d\n", x2, y2);
                fprintf(fp, "\n");
            }
            else
            {
                fprintf(fp, "%d %d\n", x1, y1);
                fprintf(fp, "%d %d\n", x1, y2);
                fprintf(fp, "%d %d\n", x2, y2);
                fprintf(fp, "\n");
            }
        }
        else if(routetype==ZROUTE)
        {
            Zpoint = treeedge->route.Zpoint;
            if(treeedge->route.HVH)
            {
                fprintf(fp, "%d %d\n", x1, y1);
                fprintf(fp, "%d %d\n", Zpoint, y1);
                fprintf(fp, "%d %d\n", Zpoint, y2);
                fprintf(fp, "%d %d\n", x2, y2);
                fprintf(fp, "\n");
            }
            else
            {
                fprintf(fp, "%d %d\n", x1, y1);
                fprintf(fp, "%d %d\n", x1, Zpoint);
                fprintf(fp, "%d %d\n", x2, Zpoint);
                fprintf(fp, "%d %d\n", x2, y2);
                fprintf(fp, "\n");
            }
        }
        else if(routetype==MAZEROUTE)
        {
            gridsX = treeedge->route.gridsX;
            gridsY = treeedge->route.gridsY;
            for(j=0; j<=treeedge->route.routelen; j++)
            {
                fprintf(fp, "%d %d\n", gridsX[j], gridsY[j]);
            }
            fprintf(fp, "\n");
        }
}
    }
    
    fprintf(fp, "%d %d\n", xmin-2, ymin-2);
    fprintf(fp, "\n");
    fprintf(fp, "%d %d\n", xmax+2, ymax+2);
    fclose(fp);
}

void getlen()
{
    int i, edgeID, totlen=0;
    TreeEdge *treeedge;
    TreeNode *treenodes;
    
    for(i=0; i<numValidNets; i++)
    {
        treenodes = sttrees[i].nodes;
        for(edgeID=0; edgeID<2*sttrees[i].deg-3; edgeID++)
        {
            treeedge = &(sttrees[i].edges[edgeID]);
            if(treeedge->route.type<MAZEROUTE)
                printf("wrong\n");
//                totlen += ADIFF(treenodes[treeedge->n1].x, treenodes[treeedge->n2].x) + ADIFF(treenodes[treeedge->n1].y, treenodes[treeedge->n2].y);
            else
                totlen += treeedge->route.routelen;
        }
    }
    printf("Routed len: %d\n", totlen);
}


void ConvertToFull3DType2 () 
{
	short *gridsX,*gridsY,*gridsL, tmpX[MAXLEN], tmpY[MAXLEN],tmpL[MAXLEN];
	int k, netID, edgeID, routeLen, n1a, n2a;
	int newCNT, numVIA,deg, j;
	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;

	numVIA = 0;
	

	for(netID=0;netID<numValidNets;netID++)
	{
		treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		treenodes=sttrees[netID].nodes;

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {

				newCNT = 0;
				routeLen =  treeedge->route.routelen;
//				printf("netID %d, edgeID %d, len %d\n",netID, edgeID, routeLen);
				n1a = treeedge->n1a;
				n2a = treeedge->n2a;
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;
				gridsL = treeedge->route.gridsL;
/*
				if (edgeID == treenodes[n1a].hID) {
					for (k = treenodes[n1a].botL; k < treenodes[n1a].topL; k++) {
						tmpX[newCNT] = gridsX[0];
						tmpY[newCNT] = gridsY[0];
						tmpL[newCNT] = k;
						newCNT++;
						numVIA++;
					}
				}
				*/

				//finish from n1->real route

				for (j = 0; j< routeLen; j ++)
				{
					tmpX[newCNT] = gridsX[j];
					tmpY[newCNT] = gridsY[j];
					tmpL[newCNT] = gridsL[j];
					newCNT++;
						
					if (gridsL[j] > gridsL[j+1]) {
						for (k = gridsL[j]; k > gridsL[j+1]; k--) {
							tmpX[newCNT] = gridsX[j+1];
							tmpY[newCNT] = gridsY[j+1];
							tmpL[newCNT] = k;
							newCNT++;
							numVIA++;
						}
					} else if (gridsL[j] < gridsL[j+1]){
						for (k = gridsL[j]; k < gridsL[j+1]; k++) {
							tmpX[newCNT] = gridsX[j+1];
							tmpY[newCNT] = gridsY[j+1];
							tmpL[newCNT] = k;
							newCNT++;
							numVIA++;
						}
					}
				}
				tmpX[newCNT] = gridsX[j];
				tmpY[newCNT] = gridsY[j];
				tmpL[newCNT] = gridsL[j];
				newCNT++;

				/*
				if (edgeID == treenodes[n2a].hID) {
					if (treenodes[n2a].topL != treenodes[n2a].botL)
					for (k = treenodes[n2a].topL-1; k >= treenodes[n2a].botL; k--) {
						tmpX[newCNT] = gridsX[routeLen];
						tmpY[newCNT] = gridsY[routeLen];
						tmpL[newCNT] = k;
						newCNT++;
						numVIA++;
					}
				}
				*/
				// last grid -> node2 finished

				if(treeedges[edgeID].route.type==MAZEROUTE)
				{
					free(treeedges[edgeID].route.gridsX);
					free(treeedges[edgeID].route.gridsY);
					free(treeedges[edgeID].route.gridsL);
				}
				treeedge->route.gridsX = (short*)calloc(newCNT, sizeof(short));
				treeedge->route.gridsY = (short*)calloc(newCNT, sizeof(short));
				treeedge->route.gridsL = (short*)calloc(newCNT, sizeof(short));
				treeedge->route.type = MAZEROUTE;
				treeedge->route.routelen = newCNT-1;

				for(k=0; k<newCNT; k++)
				{
					treeedge->route.gridsX[k] = tmpX[k];
					treeedge->route.gridsY[k] = tmpY[k];
					treeedge->route.gridsL[k] = tmpL[k];

				}
			}
//			printEdge3D(netID, edgeID);
		}
	}
//	printf("Total number of via %d\n",numVIA);

}

void netpinOrderInc()
{
	int i, j, d, n1, n2, x1, y1, x2, y2, ind, l, totalLength,xmin;
	TreeEdge *treeedges, *treeedge ;
    TreeNode *treenodes;
	StTree* stree;

	float npvalue;

	numTreeedges = 0;
	for(j=0; j<numValidNets; j++)
    {
		d = sttrees[j].deg;
		numTreeedges += 2*d -3;
	}

	if (treeOrderPV!=NULL) {
		free(treeOrderPV);
	}

	treeOrderPV = (OrderNetPin*) malloc(numValidNets*sizeof(OrderNetPin));


	i = 0;
	for(j=0; j<numValidNets; j++)
	{
		xmin = BIG_INT;
		totalLength = 0;
		treenodes=sttrees[j].nodes;
		stree = &(sttrees[j]);
		d = stree->deg;
		for(ind=0; ind<2*d-3; ind++)
		{
			totalLength += stree->edges[ind].len;
			if (xmin < treenodes[stree->edges[ind].n1].x) {
				xmin = treenodes[stree->edges[ind].n1].x;
			}

		}

		npvalue = (float) totalLength; //TODO: Michael: This probably could be used

		treeOrderPV[j].npv = npvalue;
		treeOrderPV[j].treeIndex = j;
		treeOrderPV[j].minX = xmin;
	}

	qsort (treeOrderPV, numValidNets, sizeof(OrderNetPin), comparePVMINX);
	qsort (treeOrderPV, numValidNets, sizeof(OrderNetPin), comparePVPV);
}


void SPRoute::fillVIA()
{
	short tmpX[MAXLEN], tmpY[MAXLEN],*gridsX,*gridsY,*gridsL, tmpL[MAXLEN];
	int i, k, netID, edgeID, routeLen, n1a, n2a;
	int n1, n2,  newCNT, numVIAT1, numVIAT2,deg, j;
	Route *route;
	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;

	numVIAT1 = 0;
	numVIAT2 = 0;
	

	for(netID=0;netID<numValidNets;netID++)
	{
		treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		treenodes=sttrees[netID].nodes;

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {

				newCNT = 0;
				routeLen =  treeedge->route.routelen;
//				printf("netID %d, edgeID %d, len %d\n",netID, edgeID, routeLen);
				n1 = treeedge->n1;
				n2 = treeedge->n2;
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;
				gridsL = treeedge->route.gridsL;

				
				n1a = treenodes[n1].stackAlias;

				
				n2a = treenodes[n2].stackAlias;

				n1a = treeedge->n1a;
				n2a = treeedge->n2a;

	
		
				if (edgeID == treenodes[n1a].hID || edgeID == treenodes[n2a].hID) {

					if (edgeID == treenodes[n1a].hID) {

						for (k = treenodes[n1a].botL; k < treenodes[n1a].topL; k++) {
							tmpX[newCNT] = gridsX[0];
							tmpY[newCNT] = gridsY[0];
							tmpL[newCNT] = k;
							newCNT++;
							if (n1a < deg) {
								numVIAT1++;
							} else {
								numVIAT2 ++;
							}
						}
					}

					//finish from n1->real route

					for (j = 0; j< routeLen; j ++)
					{
						tmpX[newCNT] = gridsX[j];
						tmpY[newCNT] = gridsY[j];
						tmpL[newCNT] = gridsL[j];
						newCNT++;
							
/*						if (gridsL[j] > gridsL[j+1]) {
							printf("fill via should not entered\n");
							for (k = gridsL[j]; k > gridsL[j+1]; k--) {
								tmpX[newCNT] = gridsX[j+1];
								tmpY[newCNT] = gridsY[j+1];
								tmpL[newCNT] = k;
								newCNT++;
								numVIA++;
							}
						} else if (gridsL[j] < gridsL[j+1]){
							printf("fill via should not entered\n");
							for (k = gridsL[j]; k < gridsL[j+1]; k++) {
								tmpX[newCNT] = gridsX[j+1];
								tmpY[newCNT] = gridsY[j+1];
								tmpL[newCNT] = k;
								newCNT++;
								numVIA++;
							}
						}
						*/
					}
					tmpX[newCNT] = gridsX[j];
					tmpY[newCNT] = gridsY[j];
					tmpL[newCNT] = gridsL[j];
					newCNT++;


				
					if (edgeID == treenodes[n2a].hID) {
						if (treenodes[n2a].topL != treenodes[n2a].botL)
						for (k = treenodes[n2a].topL-1; k >= treenodes[n2a].botL; k--) {
							tmpX[newCNT] = gridsX[routeLen];
							tmpY[newCNT] = gridsY[routeLen];
							tmpL[newCNT] = k;
							newCNT++;
							if (n2a < deg) {
								numVIAT1++;
							} else {
								numVIAT2 ++;
							}
						}
					}
					// last grid -> node2 finished

					if(treeedges[edgeID].route.type==MAZEROUTE)
					{
						free(treeedges[edgeID].route.gridsX);
						free(treeedges[edgeID].route.gridsY);
						free(treeedges[edgeID].route.gridsL);
					}
					treeedge->route.gridsX = (short*)calloc(newCNT, sizeof(short));
					treeedge->route.gridsY = (short*)calloc(newCNT, sizeof(short));
					treeedge->route.gridsL = (short*)calloc(newCNT, sizeof(short));
					treeedge->route.type = MAZEROUTE;
					treeedge->route.routelen = newCNT-1;

					for(k=0; k<newCNT; k++)
					{
						treeedge->route.gridsX[k] = tmpX[k];
						treeedge->route.gridsY[k] = tmpY[k];
						treeedge->route.gridsL[k] = tmpL[k];

					}
				}//if edgeID == treenodes[n1a].hID || edgeID == treenodes[n2a].hID
			}
//			printEdge3D(netID, edgeID);
		}
	}
	if(verbose_ > none) {
		printf("via related to pin nodes %d\n",numVIAT1);
		printf("via related stiner nodes %d\n",numVIAT2);
	}

}

int threeDVIA ()
{
	short *gridsL;
	int netID, d, k, edgeID,nodeID,deg, numpoints, n1, n2, corN;
	int routeLen, n1a, n2a, numVIA, j;
	TreeEdge *treeedges, *treeedge;
	TreeNode *treenodes;
	
	numVIA = 0;

	for(netID=0;netID<numValidNets;netID++)
	{
		treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		treenodes=sttrees[netID].nodes;

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{

			treeedge = &(treeedges[edgeID]);

			if (treeedge->len > 0) {

				routeLen =  treeedge->route.routelen;
				gridsL = treeedge->route.gridsL;

				for (j = 0; j< routeLen; j ++)
				{
					if (gridsL[j]!= gridsL[j+1]) {
						numVIA++;
					}
				}
			}
		}
	}

	//printf("num of vias %d\n",numVIA);
	return (numVIA);
}

void assignEdge(int netID, int edgeID, bool processDIR) 
{

	short  *gridsX, *gridsY, *gridsL;
	int i, j, k , l, length, grid, min_x, min_y, routelen, n1a, n2a, last_layer;
	int min_result, endLayer;
	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;
	bool debug = false;

	treeedges = sttrees[netID].edges;
	treenodes = sttrees[netID].nodes;
	treeedge = &(treeedges[edgeID]);

	gridsX = treeedge->route.gridsX;
	gridsY = treeedge->route.gridsY;
	gridsL = treeedge->route.gridsL;

	
	routelen = treeedge->route.routelen;
	n1a = treeedge->n1a;
	n2a = treeedge->n2a;

	for (l = 0; l < numLayers; l ++) {
		for (k = 0; k <= routelen; k ++) {
			gridD[l][k] = 100 * BIG_INT;
			viaLink[l][k] = BIG_INT;
		}
	}

	//cout << " assigned edge is called" << endl;

	if(routelen > MAXLEN)
	{
		cout << " routing length larger than MAXLEN!" << endl;
		exit(1);
	}

	for (k = 0; k < routelen; k ++) {
		if (gridsX[k] == gridsX[k+1]) {  //vertical wire
			min_y = min(gridsY[k], gridsY[k+1]);
			for (l = 0; l < numLayers; l ++) { //TODO: probably need to ban metal1
				grid = l* gridV + min_y*xGrid+gridsX[k];
				if(v_edges3D[grid].cap > 0)
					layerGrid[l][k] = v_edges3D[grid].cap - v_edges3D[grid].usage; //available capacity
				else
					layerGrid[l][k] = -10; //cap = 0, OBS or wrong direction, don't use
			}
		} else {
			min_x =  min(gridsX[k], gridsX[k+1]);   //horizontal wire
			for (l = 0; l < numLayers; l ++) {
				grid = l* gridH + gridsY[k]*(xGrid -1)+min_x;

				if(h_edges3D[grid].cap > 0) 
					layerGrid[l][k] = h_edges3D[grid].cap - h_edges3D[grid].usage;
				else
					layerGrid[l][k] = -10;
			}
		}
	}

	if (processDIR) {
		if (treenodes[n1a].assigned) {

			for (l = treenodes[n1a].botL ; l <= treenodes[n1a].topL ; l ++) {
				gridD[l][0] = 0;
			}
		} else {
			printf("warning, start point not assigned\n");
			fflush(stdout);
		}

		for (k = 0; k < routelen; k ++) {
			for (l = 0 ; l < numLayers; l ++) {
				for (i = 0 ; i < numLayers; i++) {
					if (k == 0) {
						if (l != i) {
							if (gridD[i][k] > gridD[l][k]  + ADIFF(i,l)*2) {
								gridD[i][k] = gridD[l][k]  + ADIFF(i,l)*2;
								viaLink[i][k] = l;
							}
						}
					} else {
						if (l != i) {
							if (gridD[i][k] > gridD[l][k] + ADIFF(i,l) * 3 ) {
								gridD[i][k] = gridD[l][k] + ADIFF(i,l) * 3  ;
								viaLink[i][k] = l;
							}
						}
					}
				}
			}
			for (l = 0; l < numLayers; l ++) {
				if (layerGrid[l][k] > 0) { //has capacity
					gridD[l][k+1] = gridD[l][k] + 1;
				} 
				else {
					//Michael: overflow = - layerGrid[l][k], one more overflow increase one more BIG_INT
					//when OBS or cap = 0, overflow is considered 10
					
					gridD[l][k+1] = gridD[l][k] + (-1 * layerGrid[l][k] + 1) * BIG_INT; 
				}
			}
		}
		

		for (l = 0 ; l < numLayers; l ++) {
			for (i = 0 ; i < numLayers; i++) {
				if (l != i) {
					if (gridD[i][k] > gridD[l][k] + ADIFF(i,l)*1){//+ ADIFF(i,l) * 3 ) {
						gridD[i][k] = gridD[l][k] + ADIFF(i,l)*1;//+ ADIFF(i,l) * 3 ;
						viaLink[i][k] = l;
					}
				}
			}
		}

		/*if(netID == 1794 && edgeID == 0)
		{
			cout << "net: " << nets[netID]->name << " gridD:" << endl;
			cout << "routelen: " << routelen << endl;
			for(int routelen_k = 0; routelen_k <= routelen; routelen_k ++)
			{
				cout << " (" << gridsX[routelen_k] << "," <<  gridsY[routelen_k] <<") " ;
				cout << h_edges[gridsY[routelen_k]*(xGrid-1)+gridsX[routelen_k]].usage << "/" << h_edges[gridsY[routelen_k]*(xGrid -1)+gridsX[routelen_k]].cap << " ";
			}
			cout << endl;
			for(int layer = numLayers - 1; layer >= 0; layer--)
			{
				for(int routelen_k = 0; routelen_k <= routelen; routelen_k ++)
				{
					int tmpGrid = layer* gridH + gridsY[routelen_k]*(xGrid -1)+gridsX[routelen_k];
					cout << " " << gridD[layer][routelen_k] << "(" <<  h_edges3D[tmpGrid].usage << "/" << h_edges3D[tmpGrid].cap <<")";
				}
				cout << endl;
			}
		}*/


		k = routelen;


		if (treenodes[n2a].assigned) {
			min_result = 100*BIG_INT;
			endLayer = treenodes[n2a].topL;
			for (i = treenodes[n2a].topL; i >= treenodes[n2a].botL; i--) {
				if (gridD[i][routelen] < min_result) {
					min_result = gridD[i][routelen] ;
					endLayer = i;
				}
			}
		} else {
			min_result = gridD[0][routelen];
			endLayer = 0;
			for (i = 0; i < numLayers; i++) {
				if (gridD[i][routelen] < min_result) {
					min_result = gridD[i][routelen] ;
					endLayer = i;
				}
			}
		}

		k = routelen;

		/*if(netID == 1794 && edgeID == 0) {
			cout << " endLayer: " << endLayer << endl;
			string tmpOut = (treenodes[n2a].assigned == true)? "1" : "0";
			cout << "treenodes[n2a].assigned: " << tmpOut << " topL: " << treenodes[n2a].topL << " botL: " << treenodes[n2a].botL<< endl;
		}*/

		if(endLayer >= numLayers || endLayer < 0) {
			cout << "endlayer exceeds at netID: " << netID << " edgeID: " << edgeID << endl;
			exit(1);
		}

		if (viaLink[endLayer][routelen] == BIG_INT) {

			last_layer = endLayer;
			//printf("endlayer: %d\n", last_layer);
		} else {
			last_layer = viaLink[endLayer][routelen];
			//printf("vialink last layer: %d\n", last_layer);
		}


		for (k = routelen ; k >= 0; k --) {
			gridsL[k] = last_layer;
			if (viaLink[last_layer][k] == BIG_INT) {
				last_layer = last_layer;
			} else {
				last_layer = viaLink[last_layer][k];
			}
		}

		if (gridsL[0] < treenodes[n1a].botL ) {
			treenodes[n1a].botL = gridsL[0];
			treenodes[n1a].lID = edgeID;
		}
		if (gridsL[0] > treenodes[n1a].topL) {
			treenodes[n1a].topL = gridsL[0];
			treenodes[n1a].hID = edgeID;
		}


		k = routelen;
		if (treenodes[n2a].assigned) {
			if (gridsL[routelen] < treenodes[n2a].botL ) {
				treenodes[n2a].botL = gridsL[routelen];
				treenodes[n2a].lID = edgeID;
			}
			if (gridsL[routelen] > treenodes[n2a].topL) {
				treenodes[n2a].topL = gridsL[routelen];
				treenodes[n2a].hID = edgeID;
			}

		} else {
			//treenodes[n2a].assigned = true;
			treenodes[n2a].topL = gridsL[routelen];//max(endLayer, gridsL[routelen]);
			treenodes[n2a].botL = gridsL[routelen];//min(endLayer, gridsL[routelen]);
			treenodes[n2a].lID = treenodes[n2a].hID = edgeID;
		}


		if (treenodes[n2a].assigned) {
			if (gridsL[routelen] > treenodes[n2a].topL || gridsL[routelen] < treenodes[n2a].botL ) {
				printf("target ending layer out of range\n");
			}			

		}

	} else {

		if (treenodes[n2a].assigned) {
			for (l = treenodes[n2a].botL ; l <= treenodes[n2a].topL ; l ++) {
				gridD[l][routelen] = 0;
			}
		} 

		for (k = routelen; k > 0; k --) {
			for (l = 0 ; l < numLayers; l ++) {
				for (i = 0 ; i < numLayers; i++) {
					if (k == routelen) {
						if (l != i) {
							if (gridD[i][k] > gridD[l][k] + ADIFF(i,l)*2) {
								gridD[i][k] = gridD[l][k] + ADIFF(i,l)*2 ;
								viaLink[i][k] = l;
							}
						}
					} else {
						if (l != i) {
							if (gridD[i][k] > gridD[l][k] + ADIFF(i,l) * 3 ) {
								gridD[i][k] = gridD[l][k] + ADIFF(i,l) * 3 ;
								viaLink[i][k] = l;
							}
						}
					}
				}
			}
			for (l = 0; l < numLayers; l ++) {
				if (layerGrid[l][k-1] > 0) {
					gridD[l][k-1] = gridD[l][k]+1;
				} 
				else  {
					gridD[l][k-1] = gridD[l][k] + (-1 * layerGrid[l][k - 1] + 1) * BIG_INT;
				}

			}
		}
		
		
		for (l = 0 ; l < numLayers; l ++) {
			for (i = 0 ; i < numLayers; i++) {
				if (l != i) {
					if (gridD[i][0] > gridD[l][0] + ADIFF(i,l)*1) {
						gridD[i][0] = gridD[l][0] + ADIFF(i,l)*1;
						viaLink[i][0] = l;
					}
				}
			}
		}

		if (treenodes[n1a].assigned) {
			min_result = BIG_INT;
			for (i = treenodes[n1a].topL; i >= treenodes[n1a].botL; i--) {
				if (gridD[i][k] < min_result) {
					min_result = gridD[i][0] ;
					endLayer = i;
				}
			}
			
		} else {
			min_result = gridD[0][k];
			endLayer = 0;
			for (i = 0; i < numLayers; i++) {
				if (gridD[i][k] < min_result) {
					min_result = gridD[i][k] ;
					endLayer = i;
				}
			}
		}

		last_layer = endLayer;

		for (k = 0 ; k <= routelen ; k ++) {
			if (viaLink[last_layer][k] == BIG_INT) {
				last_layer = last_layer;
			} else {
				last_layer = viaLink[last_layer][k];
			}
			gridsL[k] = last_layer;
		}

		gridsL[routelen] = gridsL[routelen -1];

		if (gridsL[routelen] < treenodes[n2a].botL ) {
			treenodes[n2a].botL = gridsL[routelen];
			treenodes[n2a].lID = edgeID;

		}
		if (gridsL[routelen] > treenodes[n2a].topL) {
			treenodes[n2a].topL = gridsL[routelen];
			treenodes[n2a].hID = edgeID;
		}

		if (treenodes[n1a].assigned) {

			if (gridsL[0] < treenodes[n1a].botL ) {
				treenodes[n1a].botL = gridsL[0];
				treenodes[n1a].lID = edgeID;
			}
			if (gridsL[0] > treenodes[n1a].topL) {
				treenodes[n1a].topL = gridsL[0];
				treenodes[n1a].hID = edgeID;
			}

		} else {
			//treenodes[n1a].assigned = true;
			treenodes[n1a].topL = gridsL[0];//max(endLayer, gridsL[0]);
			treenodes[n1a].botL = gridsL[0];//min(endLayer, gridsL[0]);
			treenodes[n1a].lID = treenodes[n1a].hID =edgeID;
		}
	}
	treeedge->assigned = true;

	for (k = 0; k < routelen; k ++) {
		if (gridsX[k] == gridsX[k+1]) {
			min_y = min(gridsY[k], gridsY[k+1]);
			grid = gridsL[k]* gridV + min_y*xGrid+gridsX[k];
			

			if (v_edges3D[grid].usage < v_edges3D[grid].cap) {
				v_edges3D[grid].usage++;
				
			} else {
				v_edges3D[grid].usage++;
				/*cout << "here v exceeds" << endl;
				string tmpOut = (processDIR == true)? "1" : "0";
				cout << " netID : " << netID << " edgeID: " << edgeID << " processDIR: " << tmpOut <<  endl;
				exit(1);*/
			}
			if(v_edges3D[grid].cap == 0) {
				//cout << "cap 0 edge used: netID " << netID << " " << edgeID << " grid: " grid << " layer: "  << gridsL[k]<< endl;
				debug = true;
			}
				
		} else {
			min_x =  min(gridsX[k], gridsX[k+1]);
			grid = gridsL[k]* gridH + gridsY[k]*(xGrid -1)+min_x;
			
			if (h_edges3D[grid].usage < h_edges3D[grid].cap) {
				h_edges3D[grid].usage ++;
			} else {
				h_edges3D[grid].usage ++;
				/*cout << "here h exceeds" << endl;
				string tmpOut = (processDIR == true)? "1" : "0";
				cout << " netID : " << netID << " edgeID: " << edgeID << " processDIR: " <<  tmpOut << endl;
				exit(1);*/
			}
			if(h_edges3D[grid].cap == 0) {
				//cout << "cap 0 edge used: netID " << netID << " " << edgeID << " grid: " grid << " layer: "  << gridsL[k]<< endl;
				debug = true;
			}
		}
	}

	/*if(debug) {
		cout << "netID: " << netID << " netname: " << nets[netID]->name << " edgeID: " << edgeID << endl;
		for (int k = 0; k < routelen; k ++) {
			cout << "k " << k << " : ( " << gridsX[k] << ", " << gridsY[k] << " ) " ;
			for(int i = 0; i < numLayers; i++) {
				cout << gridD[i][k] << " ";
			}
			cout << endl;
		}
		exit(1);
	}*/
	
}

void SPRoute::newLayerAssignmentV4() 
{
	bool assigned, inOne, BIdir;
	short *gridsX, *gridsY, *gridsL;
	int i, j,k, l, netID, edgeID, nodeID, routeLen, min_y, min_x;
	int grid,numError, preFH, preFV, n1, n2, connectionCNT, deg, tmpLayer;

	int zigP[MAXLEN], dirZig[MAXLEN],numZig, curX, nextX, preD, curD, zigHead, zigTail, preBH, preBV, n1a, n2a;
	int quehead, quetail, numNodes;
	int edgeQueue[50000];
	int nodeIndex[50000];
	int trash;
	int sumcheck = 0;

	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;
	
	numError = 0;

	for (netID = 0; netID < numValidNets; netID++) {
		treeedges = sttrees[netID].edges;
		treenodes = sttrees[netID].nodes;
		deg = sttrees[netID].deg;
		for (edgeID = 0; edgeID < 2*deg-3; edgeID++) {

			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {

				routeLen = treeedge->route.routelen;
				treeedge->route.gridsL = (short*)calloc((routeLen+1), sizeof(short));
				treeedge->assigned = false;
				

			}
		}
	}
	netpinOrderInc();

	for(i=0; i<numValidNets; i++)
    {
    	//if(i % 10000 == 0)
    	//	cout << "LA nets:" << i << endl;

		netID = treeOrderPV[i].treeIndex;
		treeedges = sttrees[netID].edges;
		treenodes = sttrees[netID].nodes;
		deg = sttrees[netID].deg;
		quehead = quetail = 0;
		numNodes = 2*deg-2;

		if(verbose_ > none && deg > 1000)
			cout << "LA large net" << endl;

		for (nodeID = 0; nodeID < deg; nodeID ++) {
			for (k = 0; k < treenodes[nodeID].conCNT; k++) {
				edgeID = treenodes[nodeID].eID[k];
				if (!treeedges[edgeID].assigned) {
					edgeQueue[quetail] = edgeID;
					treeedges[edgeID].assigned = true;
					quetail++;
				}
			}
		}

		while (quehead != quetail) {
			edgeID = edgeQueue[quehead];
			treeedge = &(treeedges[edgeID]);
			sumcheck += treeedge->route.routelen;
			if (treenodes[treeedge->n1a].assigned) {
				assignEdge(netID, edgeID, 1);
				treeedge->assigned = true;
				if (!treenodes[treeedge->n2a].assigned) {
					for (k = 0; k < treenodes[treeedge->n2a].conCNT; k++) {
						edgeID = treenodes[treeedge->n2a].eID[k];
						if (!treeedges[edgeID].assigned) {
							edgeQueue[quetail] = edgeID;
							treeedges[edgeID].assigned = true;
							quetail++;
						}
					}
					treenodes[treeedge->n2a].assigned = true;
				}
			} else {
				assignEdge(netID, edgeID, 0);
				treeedge->assigned = true;
				if (!treenodes[treeedge->n1a].assigned) {
					for (k = 0; k < treenodes[treeedge->n1a].conCNT; k++) {
						edgeID = treenodes[treeedge->n1a].eID[k];
						if (!treeedges[edgeID].assigned) {
							edgeQueue[quetail] = edgeID;
							treeedges[edgeID].assigned = true;
							quetail++;
						}
					}
					treenodes[treeedge->n1a].assigned = true;
				}
			}
			quehead++;
		}

		deg = sttrees[netID].deg;


		for(nodeID=0;nodeID<2*deg-2;nodeID++)
		{
			treenodes[nodeID].topL = -1;
			treenodes[nodeID].botL = numLayers;
			treenodes[nodeID].conCNT = 0;
			treenodes[nodeID].hID = BIG_INT;
			treenodes[nodeID].lID = BIG_INT;
			treenodes[nodeID].status = 0;
			treenodes[nodeID].assigned = false;


			if(nodeID<deg)
			{
				
				//treenodes[nodeID].botL = 0;
				treenodes[nodeID].topL = treenodes[nodeID].layer;
				treenodes[nodeID].botL = treenodes[nodeID].layer;
				treenodes[nodeID].assigned = true;
				treenodes[nodeID].status = 1;
			}

		}

		for (edgeID = 0; edgeID < 2*deg-3; edgeID++) {

			treeedge = &(treeedges[edgeID]);

			if (treeedge->len > 0) {

				routeLen = treeedge->route.routelen;

				n1 = treeedge->n1;
				n2 = treeedge->n2;
				gridsL = treeedge->route.gridsL;

				n1a = treenodes[n1].stackAlias;
				n2a = treenodes[n2].stackAlias;
				connectionCNT = treenodes[n1a].conCNT;
				treenodes[n1a].heights[connectionCNT] = gridsL[0];
				treenodes[n1a].eID[connectionCNT] = edgeID;
				treenodes[n1a].conCNT++;
				

				if (gridsL[0]>treenodes[n1a].topL) {
					treenodes[n1a].hID = edgeID;
					treenodes[n1a].topL = gridsL[0];
				}
				if (gridsL[0]<treenodes[n1a].botL) {
					treenodes[n1a].lID = edgeID;
					treenodes[n1a].botL = gridsL[0];
				}

				//if(string(nets[netID]->name) == "ionet11")
				//	cout << " medium x y l:" << treenodes[n1a].x << " " << treenodes[n1a].y << " " << treenodes[n1a].botL  << " " << treenodes[n1a].topL<< endl;
				
				treenodes[n1a].assigned = true;

				connectionCNT = treenodes[n2a].conCNT;
				treenodes[n2a].heights[connectionCNT] = gridsL[routeLen];
				treenodes[n2a].eID[connectionCNT] = edgeID;
				treenodes[n2a].conCNT++;
				if (gridsL[routeLen]>treenodes[n2a].topL) {
					treenodes[n2a].hID = edgeID;
					treenodes[n2a].topL = gridsL[routeLen];
				} 
				if (gridsL[routeLen]<treenodes[n2a].botL) {
					treenodes[n2a].lID = edgeID;
					treenodes[n2a].botL = gridsL[routeLen];
				}

				//if(string(nets[netID]->name) == "ionet11")
				//	cout << " medium x y l:" << treenodes[n2a].x << " " << treenodes[n2a].y << " " << treenodes[n2a].botL  << " " << treenodes[n2a].topL<< endl;

				treenodes[n2a].assigned = true;
			
			}//edge len > 0
		} // eunmerating edges 

		if(verbose_ > none && deg > 1000)
			cout << "LA large net finished" << endl;

	}

	//printf("sum check number 2 %d\n",sumcheck);
}


int findLayer(int netID, TreeNode treenode)
{
	for(int i = 0; i < nets[netID]->deg; i++)
	{
		if(treenode.x == nets[netID]->pinX[i] && treenode.y == nets[netID]->pinY[i])
		{
			//if(string(nets[netID]->name) == "iopin242")
			//	cout << "FOUND " << treenode.x << " " << treenode.y << " on layer " << nets[netID]->pinL[i] << endl;
			return nets[netID]->pinL[i] - 1;
		}
	}
	cout << "UNABLE TO FIND LAYER: " << nets[netID]->name << " " << treenode.x << " " << treenode.y << endl;
	exit(1);
}

void SPRoute::newLA ()
{
	int netID, i,d, k, edgeID,nodeID,deg, numpoints, n1, n2, corN,  tmpX[MAXLEN], tmpY[MAXLEN],*gridsX,*gridsY,*gridsL, tmpL[MAXLEN], routeLen, n1a, n2a;;
	int n1x, n1y, n1l, n2x, n2y, n2l,  grid, numError, preH, preV, min_y, min_x,connectionCNT;
	Route *route;
	bool redundant, newCNT, numVIA, j, l, distance;;
	TreeEdge *treeedges, *treeedge;
	TreeNode *treenodes;
	bool assigned, inOne;
	int zigP[MAXLEN], dirZig[MAXLEN],numZig, curX, nextX, preD, curD,tmpLayer;

	for(netID=0;netID<numValidNets;netID++)
	{
		treeedges=sttrees[netID].edges;
		treenodes=sttrees[netID].nodes;
		deg=sttrees[netID].deg;

		numpoints=0;

		for(d=0;d<2*deg-2;d++)
		{
			treenodes[d].topL = -1;
			treenodes[d].botL = numLayers;
			//treenodes[d].l = 0;
			treenodes[d].assigned = false;
			treenodes[d].stackAlias = d;
			treenodes[d].conCNT = 0;
			treenodes[d].hID = BIG_INT;
			treenodes[d].lID = BIG_INT;
			treenodes[d].status = 0;


			if(d<deg)
			{
				treenodes[d].layer = findLayer(netID, treenodes[d]);
				//treenodes[d].botL = treenodes[d].topL = 0;
				treenodes[d].botL = treenodes[d].layer; //0; //netID, d = pinID
				treenodes[d].topL = (treenodes[d].layer == 0)? 2 : treenodes[d].layer; //Michael
				//if(string(nets[netID]->name) == "ionet11")
				//	cout << " x y l:" << treenodes[d].x << " " << treenodes[d].y << " " << treenodes[d].botL  << " " << treenodes[d].topL<< endl;
				//treenodes[d].l = 0;
				treenodes[d].assigned = true;
				treenodes[d].status = 1;
			
				xcor[numpoints]=treenodes[d].x;
				ycor[numpoints]=treenodes[d].y;
				dcor[numpoints]=d;
				numpoints++;
			} else {
				redundant=false;

				for(k=0;k<numpoints;k++)
				{
					if((treenodes[d].x==xcor[k])&&(treenodes[d].y==ycor[k]))
					{
						treenodes[d].stackAlias = dcor[k];
						
						redundant=true;
						break;
					}
				}
				if(!redundant)
				{
					xcor[numpoints]=treenodes[d].x;
					ycor[numpoints]=treenodes[d].y;
					dcor[numpoints]=d;
					numpoints++;
				}
			}
		}
	}

	for(netID=0;netID<numValidNets;netID++)
	{
		treeedges=sttrees[netID].edges;
		treenodes=sttrees[netID].nodes;
		deg = sttrees[netID].deg;

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {

				n1 = treeedge->n1;
				n2 = treeedge->n2;

				treeedge->n1a = treenodes[n1].stackAlias;
				treenodes[treeedge->n1a ].eID[treenodes[treeedge->n1a ].conCNT] = edgeID;
				treenodes[treeedge->n1a ].conCNT++;
				treeedge->n2a =  treenodes[n2].stackAlias;
				treenodes[treeedge->n2a].eID[treenodes[treeedge->n2a].conCNT] = edgeID;
				treenodes[treeedge->n2a].conCNT++;
			}
		}
	}

	if(verbose_ > none) {
		printf("node processing\n");
	}
	newLayerAssignmentV4();
	if(verbose_ > none) {
		printf("layer assignment\n");
	}
	numVIA = 0;
	ConvertToFull3DType2() ;
}

void printEdge3D(int netID, int edgeID)
{
    int i;
    TreeEdge edge;
    TreeNode *nodes;
    
    edge = sttrees[netID].edges[edgeID];
    nodes = sttrees[netID].nodes;
    
    printf("edge %d: n1 %d (%d, %d)-> n2 %d(%d, %d)\n", edgeID, edge.n1, nodes[edge.n1].x, nodes[edge.n1].y,edge.n2, nodes[edge.n2].x, nodes[edge.n2].y);
	if (edge.len > 0) {
		for(i=0; i<=edge.route.routelen; i++)
		{
			printf("(%d, %d,%d) ", edge.route.gridsX[i], edge.route.gridsY[i],edge.route.gridsL[i]);
		}
		printf("\n");
	}
}

void printTree3D(int netID)
{
	int edgeID, nodeID;
	for(nodeID=0; nodeID<2*sttrees[netID].deg-2;nodeID++)
    {
		printf("nodeID %d,  [%d, %d]\n", nodeID,sttrees[netID].nodes[nodeID].y,sttrees[netID].nodes[nodeID].x);
	}

	for(edgeID=0; edgeID<2*sttrees[netID].deg-3; edgeID++)
    {
		printEdge3D(netID, edgeID);
	}
}


void checkRoute3D()
{
	short  *gridsX, *gridsY, *gridsL;
    int i, netID, edgeID,nodeID, edgelength;
    int n1, n2, x1, y1, x2, y2, l1, l2, deg;
    int cnt, Zpoint, distance;
	bool gridFlag;
    TreeEdge *treeedge;
    TreeNode *treenodes;
    
    for(netID=0; netID<numValidNets; netID++) //TODO: This can be parallelized
    {

        treenodes = sttrees[netID].nodes;
		deg = sttrees[netID].deg;

		for(nodeID=0;nodeID<2*deg-2;nodeID++)
		{
			if(nodeID<deg)
			{
				if (treenodes[nodeID].botL != 0) {
					//printf("causing pin node floating\n");
				}

				if (treenodes[nodeID].botL  > treenodes[nodeID].topL) {
					//printf("pin node l %d h %d wrong lid %d hid %d\n", treenodes[nodeID].botL, treenodes[nodeID].topL, treenodes[nodeID].lID, treenodes[nodeID].hID);
				}
			}
		}
        for(edgeID=0; edgeID<2*sttrees[netID].deg-3; edgeID++)
        {
			if (sttrees[netID].edges[edgeID].len == 0) {
				continue;
			}
            treeedge = &(sttrees[netID].edges[edgeID]);
            edgelength = treeedge->route.routelen;
            n1 = treeedge->n1;
            n2 = treeedge->n2;
            x1 = treenodes[n1].x;
            y1 = treenodes[n1].y;
            x2 = treenodes[n2].x;
            y2 = treenodes[n2].y;
            gridsX = treeedge->route.gridsX;
            gridsY = treeedge->route.gridsY;
			gridsL = treeedge->route.gridsL;

			gridFlag = false;
            
			if(gridsX[0]!=x1 || gridsY[0]!=y1 )
            {
                printf("net[%d] edge[%d] start node wrong, net deg %d, n1 %d\n", netID, edgeID, deg, n1);
                printEdge3D(netID, edgeID);
            }
			if(gridsX[edgelength]!=x2 || gridsY[edgelength]!=y2)
            {
                printf("net[%d] edge[%d] end node wrong, net deg %d, n2 %d\n", netID, edgeID,deg, n2);
                printEdge3D(netID, edgeID);
            }
            for(i=0; i<treeedge->route.routelen; i++)
            {
				distance = ADIFF(gridsX[i+1],gridsX[i]) + ADIFF(gridsY[i+1],gridsY[i]) + ADIFF(gridsL[i+1],gridsL[i]);
                if(distance >1 || distance < 0)
                {
					gridFlag = true;
                    printf("net[%d] edge[%d] maze route wrong, distance %d, i %d\n", netID, edgeID,distance,i);
					printf("current [%d, %d, %d], next [%d, %d, %d]",gridsL[i],gridsY[i],gridsX[i],gridsL[i+1],gridsY[i+1],gridsX[i+1]);
                }	
            }

			for(i=0; i<=treeedge->route.routelen; i++) {
				if (gridsL[i] < 0) {
					printf("gridsL less than 0, %d\n",gridsL[i]);
				}
			}
			if (gridFlag) {
				printEdge3D(netID, edgeID);
			}
        }
    }
}

void write3D()
{
	short *gridsX, *gridsY, *gridsL;
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	
	fp=fopen("output.out", "w");
    if (fp == NULL) {
        printf("Error in opening %s\n", "output.out");
        exit(1);
    }

	for(netID=0;netID<numValidNets;netID++)
	{
		fprintf(fp, "%s %d\n", nets[netID]->name, netID);
		treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		

		nodes = sttrees[netID].nodes;
		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			edge = sttrees[netID].edges[edgeID];
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {
				
				routeLen = treeedge->route.routelen;
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;
				gridsL = treeedge->route.gridsL;
				lastX = wTile*(gridsX[0]+0.5)+xcorner;
				lastY = hTile*(gridsY[0]+0.5)+ycorner;
				lastL = gridsL[0];
				for (i = 1; i <= routeLen; i ++) {
					xreal = wTile*(gridsX[i]+0.5)+xcorner;
					yreal = hTile*(gridsY[i]+0.5)+ycorner;

					fprintf(fp, "(%d,%d,%d)-(%d,%d,%d)\n", lastX,lastY,lastL+1,xreal,yreal,gridsL[i]+1);

					lastX = xreal;
					lastY = yreal;
					lastL = gridsL[i];
				}
			}
		}
		fprintf(fp, "!\n");
	}
	fclose(fp);
}

void StNetOrder()
{
	short *gridsX, *gridsY;
	int i, j, d, n1, n2, x1, y1, x2, y2, ind, l, grid, min_x, min_y;
	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;
	StTree* stree;

	numTreeedges = 0;

	if (treeOrderCong != NULL) {
		free(treeOrderCong);
	}

	treeOrderCong = (OrderTree*) malloc(numValidNets*sizeof(OrderTree));

	i = 0;
	for(j=0; j<numValidNets; j++)
	{
		stree = &(sttrees[j]);
		d = stree->deg;
		treeOrderCong[j].xmin = 0;
		treeOrderCong[j].treeIndex = j;
		for(ind=0; ind<2*d-3; ind++)
		{
			treeedges = stree->edges;
            treeedge = &(treeedges[ind]);

			gridsX = treeedge->route.gridsX;
			gridsY = treeedge->route.gridsY;
			for(i=0; i<=treeedge->route.routelen; i++)
			{
				if(gridsX[i]==gridsX[i+1]) // a vertical edge
				{
					min_y = min(gridsY[i], gridsY[i+1]);
					grid = min_y*xGrid+gridsX[i];
					treeOrderCong[j].xmin += max(0, v_edges[grid].usage - v_edges[grid].cap);
				}
				else ///if(gridsY[i]==gridsY[i+1])// a horizontal edge
				{
					min_x = min(gridsX[i], gridsX[i+1]);
					grid = gridsY[i]*(xGrid-1)+min_x;
					treeOrderCong[j].xmin += max(0, h_edges[grid].usage - h_edges[grid].cap);
				}
			}

		}
	}

	qsort (treeOrderCong, numValidNets, sizeof(OrderTree), compareTEL);
}



void recoverEdge(int netID, int edgeID)
{
	short *gridsX, *gridsY, *gridsL;
    int i, k, grid, Zpoint, ymin, ymax, xmin, lv, lh, n1a, n2a, hl, bl, hid, bid, deg;
    int  connectionCNT, routeLen;
    RouteType ripuptype;
	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;


	treeedges = sttrees[netID].edges;
	treeedge = &(treeedges[edgeID]);

	routeLen = treeedge->route.routelen;


	if(treeedge->len==0) {
		printf("trying to recover an 0 length edge\n");
		exit(0);
	}

	treenodes = sttrees[netID].nodes;

	gridsX = treeedge->route.gridsX;
	gridsY = treeedge->route.gridsY;
	gridsL = treeedge->route.gridsL;

	deg =  sttrees[netID].deg;

	n1a = treeedge->n1a;
	n2a = treeedge->n2a;

	connectionCNT = treenodes[n1a].conCNT;
	treenodes[n1a].heights[connectionCNT] = gridsL[0];
	treenodes[n1a].eID[connectionCNT] = edgeID;
	treenodes[n1a].conCNT++;
	

	if (gridsL[0]>treenodes[n1a].topL) {
		treenodes[n1a].hID = edgeID;
		treenodes[n1a].topL = gridsL[0];
	}
	if (gridsL[0]<treenodes[n1a].botL) {
		treenodes[n1a].lID = edgeID;
		treenodes[n1a].botL = gridsL[0];
	}
	
	treenodes[n1a].assigned = true;

	connectionCNT = treenodes[n2a].conCNT;
	treenodes[n2a].heights[connectionCNT] = gridsL[routeLen];
	treenodes[n2a].eID[connectionCNT] = edgeID;
	treenodes[n2a].conCNT++;
	if (gridsL[routeLen]>treenodes[n2a].topL) {
		treenodes[n2a].hID = edgeID;
		treenodes[n2a].topL = gridsL[routeLen];
	} 
	if (gridsL[routeLen]<treenodes[n2a].botL) {
		treenodes[n2a].lID = edgeID;
		treenodes[n2a].botL = gridsL[routeLen];
	}

	
	treenodes[n2a].assigned = true;

	for(i=0; i<treeedge->route.routelen; i++)
	{
		if (gridsL[i] ==  gridsL[i+1]) {
			if(gridsX[i]==gridsX[i+1]) // a vertical edge
			{
				ymin = min(gridsY[i], gridsY[i+1]);
				grid = gridsL[i]*gridV+ymin*xGrid+gridsX[i];
				v_edges3D[grid].usage += 1;
			}
			else if(gridsY[i]==gridsY[i+1])// a horizontal edge
			{
				xmin = min(gridsX[i], gridsX[i+1]);
				grid = gridsL[i]*gridH+gridsY[i]*(xGrid-1)+xmin;
				h_edges3D[grid].usage += 1;
			} 
		}
	}

}

void SPRoute::checkUsage()
{
	short *gridsX, *gridsY, *gridsL, tmp_gridsX[XRANGE], tmp_gridsY[XRANGE];
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	int j, grid ,xmin,ymin, cnt;
	bool redsus;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	

	for(netID=0;netID<numValidNets;netID++)
	{
		treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		
		nodes = sttrees[netID].nodes;
		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			edge = sttrees[netID].edges[edgeID];
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {
				
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;

				redsus = true;

				while (redsus) {
					redsus = false;

					for(i=0; i<=treeedge->route.routelen; i++)
					{
						for (j = 0; j < i; j++) {
							if(gridsX[i]==gridsX[j] && gridsY[i]==gridsY[j]) // a vertical edge
							{
								cnt = 1;
								for (k = i+1; k <= treeedge->route.routelen; k++) {
									gridsX[j+cnt] = gridsX[k];
									gridsY[j+cnt] = gridsY[k];
									cnt++;
								}
								treeedge->route.routelen -= i-j;
								redsus = true;
								printf("redundant edge component discovered: netID: %d name: %s edgeid: %d ", netID, nets[netID]->name, edgeID);
								printf(" x: %d y: %d %d %d\n", gridsX[i], gridsY[i], i, j);
								i = 0;
								j=0;
							}
						}

					}
				}
			}
		}
	}
	if(verbose_ > none)
		printf("usage checked\n");
}

void netedgeOrderDec(int netID, OrderNetEdge* netEO)
{
	int j, d, numTreeedges;
 
	d = sttrees[netID].deg;
	numTreeedges = 2*d -3;

	for(j=0; j<numTreeedges; j++)
	{
		netEO[j].length = sttrees[netID].edges[j].route.routelen;
		netEO[j].edgeID = j;
	}

	qsort (netEO, numTreeedges, sizeof(OrderNetEdge), compareEdgeLen);
}

void printEdge2D(int netID, int edgeID)
{
    int i;
    TreeEdge edge;
    TreeNode *nodes;
    
    edge = sttrees[netID].edges[edgeID];
    nodes = sttrees[netID].nodes;
    
    printf("edge %d: n1 %d (%d, %d)-> n2 %d(%d, %d), routeType %d\n", edgeID, edge.n1, nodes[edge.n1].x, nodes[edge.n1].y,edge.n2, nodes[edge.n2].x, nodes[edge.n2].y, edge.route.type);
	if (edge.len > 0) {
		for(i=0; i<=edge.route.routelen; i++)
		{
			printf("(%d, %d) ", edge.route.gridsX[i], edge.route.gridsY[i]);
		}
		printf("\n");
	}
}

void printTree2D(int netID)
{
	int edgeID, nodeID;
	for(nodeID=0; nodeID<2*sttrees[netID].deg-2;nodeID++)
    {
		printf("nodeID %d,  [%d, %d]\n", nodeID,sttrees[netID].nodes[nodeID].y,sttrees[netID].nodes[nodeID].x);
	}

	for(edgeID=0; edgeID<2*sttrees[netID].deg-3; edgeID++)
    {
		printEdge2D(netID, edgeID);
	}
}



bool checkRoute2DTree(int netID)
{
	bool STHwrong, gridFlag, outrangeFlag;
	short *gridsX, *gridsY;
    int i, edgeID, edgelength;
    int n1, n2, x1, y1, x2, y2;
    int cnt, Zpoint, distance;
    TreeEdge *treeedge;
    TreeNode *treenodes;
    
	STHwrong = false;
    
    treenodes = sttrees[netID].nodes;
    //if(netID == 2nnn/b52163) return false;
    for(edgeID=0; edgeID<2*sttrees[netID].deg-3; edgeID++)
    {
		treeedge = &(sttrees[netID].edges[edgeID]);
        edgelength = treeedge->route.routelen;
        n1 = treeedge->n1;
        n2 = treeedge->n2;
        x1 = treenodes[n1].x;
        y1 = treenodes[n1].y;
        x2 = treenodes[n2].x;
        y2 = treenodes[n2].y;
        gridsX = treeedge->route.gridsX;
        gridsY = treeedge->route.gridsY;

		gridFlag = false;
		outrangeFlag = false;

		if (treeedge->len < 0) { 
			printf("rip upped edge without edge len re assignment\n");
			STHwrong = true;
		}



		if (treeedge->len > 0) {
			if ( treeedge->route.routelen < 1) { 
				//printf("%d %d .routelen %d len  %d\n",netID, edgeID, treeedge->route.routelen, treeedge->len);
				if(x1 != x2 || y1 != y2) {
					STHwrong = true;
					printf("checking failed %d roulen = 0 and two nodes are not overlapped\n", netID);
					return (true);
				}
			}
            //if(netID == 898 && edgeID == 804)
            //	printf("checking src: %d %d gridstart: %d %d dst: %d %d gridend: %d %d\n", y1, x1, gridsY[0],gridsX[0], y2, x2, gridsY[edgelength],gridsX[edgelength]);
			
			if(gridsX[0]!=x1 || gridsY[0]!=y1 )
			{
				printf("%d %d initial grid wrong y1 x1 [%d %d] , net start [%d %d] routelen %d\n ",netID, edgeID, y1,x1,gridsY[0],gridsX[0], treeedge->route.routelen );
				STHwrong = true;
			}
			if(gridsX[edgelength]!=x2 || gridsY[edgelength]!=y2 )
			{
				printf("%d %d end grid wrong y2 x2 [%d %d] , net start [%d %d] routelen %d\n ",netID, edgeID, y2,x2,gridsY[edgelength],gridsX[edgelength],treeedge->route.routelen );
				STHwrong = true;
			}
			for(i=0; i<treeedge->route.routelen; i++)
			{

				distance = ADIFF(gridsX[i+1],gridsX[i]) + ADIFF(gridsY[i+1],gridsY[i]) ;
				if(distance != 1)
				{
					printf("net[%d] netname: %s edge[%d] maze route wrong, distance %d, i %d\n", netID, nets[netID]->name, edgeID,distance,i);
					gridFlag = true;
					STHwrong = true;
				}
			}

			if (gridFlag) {
				printEdge2D(netID, edgeID);
			}
			if (STHwrong) {
				printf("checking failed %d STHwrong\n", netID);
				return (true);
			} 
		}
    }

	return (STHwrong);
}

void copyRS (void)
{
	int i,j,netID,p,q, k, edgeID, numEdges, xmin, xmax, ymin, ymax, numNodes;
	int n1, n2, x1,x2,y1,y2;

	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;

	if (sttreesBK != NULL) {
		for(netID=0; netID<numValidNets; netID++) {

			numEdges = 2 * sttreesBK[netID].deg -3;
			treeedges = sttreesBK[netID].edges;
			for(edgeID=0; edgeID<numEdges; edgeID++)
			{
				if(sttreesBK[netID].edges[edgeID].len>0)
				{
					free (sttreesBK[netID].edges[edgeID].route.gridsX);
					free (sttreesBK[netID].edges[edgeID].route.gridsY);
				}
			}
			free(sttreesBK[netID].nodes);
			free(sttreesBK[netID].edges);
		}
		free(sttreesBK);
	}

	sttreesBK = (StTree*) malloc(numValidNets*sizeof(StTree));
		
	for(netID=0; netID<numValidNets; netID++) {
		numNodes = 2*sttrees[netID].deg - 2;
		numEdges = 2 * sttrees[netID].deg - 3;
		
		sttreesBK[netID].nodes = (TreeNode*) malloc(numNodes*sizeof(TreeNode));

		for (i = 0; i < numNodes; i++) {
			sttreesBK[netID].nodes[i].x = sttrees[netID].nodes[i].x ;
			sttreesBK[netID].nodes[i].y = sttrees[netID].nodes[i].y ;
			for (j = 0; j < 3; j++) {
				sttreesBK[netID].nodes[i].nbr[j] = sttrees[netID].nodes[i].nbr[j] ;
				sttreesBK[netID].nodes[i].edge[j] = sttrees[netID].nodes[i].edge[j] ;
			}
		}
		sttreesBK[netID].deg = sttrees[netID].deg;

		sttreesBK[netID].edges = (TreeEdge*) malloc(numEdges*sizeof(TreeEdge));

		treeedges = sttrees[netID].edges;
		for(edgeID=0; edgeID<numEdges; edgeID++)
		{
			sttreesBK[netID].edges[edgeID].len = sttrees[netID].edges[edgeID].len;
			sttreesBK[netID].edges[edgeID].n1 = sttrees[netID].edges[edgeID].n1;
			sttreesBK[netID].edges[edgeID].n2 = sttrees[netID].edges[edgeID].n2;

			if(sttrees[netID].edges[edgeID].len>0) // only route the non-degraded edges (len>0)
			{
				sttreesBK[netID].edges[edgeID].route.routelen = sttrees[netID].edges[edgeID].route.routelen;
				sttreesBK[netID].edges[edgeID].route.gridsX = (short*)calloc((sttrees[netID].edges[edgeID].route.routelen+1), sizeof(short));
				sttreesBK[netID].edges[edgeID].route.gridsY = (short*)calloc((sttrees[netID].edges[edgeID].route.routelen+1), sizeof(short));

				for (i = 0; i <=sttrees[netID].edges[edgeID].route.routelen; i++ ) {
					sttreesBK[netID].edges[edgeID].route.gridsX[i] = sttrees[netID].edges[edgeID].route.gridsX[i];
					sttreesBK[netID].edges[edgeID].route.gridsY[i] = sttrees[netID].edges[edgeID].route.gridsY[i];
				}
			}
		}
	}

}


void SPRoute::copyBR ()
{
	short *gridsX, *gridsY;
	int i,j,netID,p,q, k, edgeID, numEdges, xmin, xmax, ymin, ymax, numNodes, grid, min_y, min_x;
	int n1, n2, x1,x2,y1,y2;

	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;

	if (sttreesBK != NULL) {

		if(verbose_ > none)
			printf("copy BR working\n");
	
		for(netID=0; netID<numValidNets; netID++) {
			numEdges = 2 * sttrees[netID].deg -3;
			treeedges = sttrees[netID].edges;
			for(edgeID=0; edgeID<numEdges; edgeID++)
			{
				if(sttrees[netID].edges[edgeID].len>0)
				{
					free (sttrees[netID].edges[edgeID].route.gridsX);
					free (sttrees[netID].edges[edgeID].route.gridsY);
				}
			}
			free(sttrees[netID].nodes);
			free(sttrees[netID].edges);
		}
		free(sttrees);

		sttrees = (StTree*) malloc(numValidNets*sizeof(StTree));
	
		for(netID=0; netID<numValidNets; netID++) {
			numNodes = 2 * sttreesBK[netID].deg - 2;
			numEdges = 2 * sttreesBK[netID].deg - 3;
			
			sttrees[netID].nodes = (TreeNode*) malloc(numNodes*sizeof(TreeNode));

			for (i = 0; i < numNodes; i++) {
				sttrees[netID].nodes[i].x = sttreesBK[netID].nodes[i].x ;
				sttrees[netID].nodes[i].y = sttreesBK[netID].nodes[i].y ;
				for (j = 0; j < 3; j++) {
					sttrees[netID].nodes[i].nbr[j] = sttreesBK[netID].nodes[i].nbr[j] ;
					sttrees[netID].nodes[i].edge[j] = sttreesBK[netID].nodes[i].edge[j] ;
				}
			}

			sttrees[netID].edges = (TreeEdge*) malloc(numEdges*sizeof(TreeEdge));

			sttrees[netID].deg = sttreesBK[netID].deg;

			treeedges = sttreesBK[netID].edges;
			for(edgeID=0; edgeID<numEdges; edgeID++)
			{
				sttrees[netID].edges[edgeID].len = sttreesBK[netID].edges[edgeID].len;
				sttrees[netID].edges[edgeID].n1 = sttreesBK[netID].edges[edgeID].n1;
				sttrees[netID].edges[edgeID].n2 = sttreesBK[netID].edges[edgeID].n2;

				sttrees[netID].edges[edgeID].route.type = MAZEROUTE;
				sttrees[netID].edges[edgeID].route.routelen = sttreesBK[netID].edges[edgeID].route.routelen;


				if(sttreesBK[netID].edges[edgeID].len>0) // only route the non-degraded edges (len>0)
				{
					sttrees[netID].edges[edgeID].route.type = MAZEROUTE;
					sttrees[netID].edges[edgeID].route.routelen = sttreesBK[netID].edges[edgeID].route.routelen;
					sttrees[netID].edges[edgeID].route.gridsX = (short*)calloc((sttreesBK[netID].edges[edgeID].route.routelen+1), sizeof(short));
					sttrees[netID].edges[edgeID].route.gridsY = (short*)calloc((sttreesBK[netID].edges[edgeID].route.routelen+1), sizeof(short));

					for (i = 0; i <=sttreesBK[netID].edges[edgeID].route.routelen; i++ ) {
						sttrees[netID].edges[edgeID].route.gridsX[i] = sttreesBK[netID].edges[edgeID].route.gridsX[i];
						sttrees[netID].edges[edgeID].route.gridsY[i] = sttreesBK[netID].edges[edgeID].route.gridsY[i];
						//printf("x %d y %d     ",sttrees[netID].edges[edgeID].route.gridsX[i],sttrees[netID].edges[edgeID].route.gridsY[i]);
					}
					//printf("\n");
				}
			}
		}
/*
		for(netID=0; netID<numValidNets; netID++) {
			numEdges = 2 * sttreesBK[netID].deg -3;
			treeedges = sttreesBK[netID].edges;
			for(edgeID=0; edgeID<numEdges; edgeID++)
			{
				if(sttrees[netID].edges[edgeID].len>0)
				{
					free (sttreesBK[netID].edges[edgeID].route.gridsX);
					free (sttreesBK[netID].edges[edgeID].route.gridsY);
				}
			}
			free(sttreesBK[netID].nodes);
			free(sttreesBK[netID].edges);
		}
		free(sttreesBK); */


		for(i=0; i<yGrid; i++)
		{
			for(j=0; j<xGrid-1; j++)
			{
				
				grid = i*(xGrid-1)+j;
				h_edges[grid].usage = 0;
			}
		}
		for(i=0; i<yGrid-1; i++)
		{
			for(j=0; j<xGrid; j++)
			{
				
				grid = i*xGrid+j;
				v_edges[grid].usage = 0;
			 }
		}
		for(netID=0; netID<numValidNets; netID++) {
			numEdges = 2 * sttrees[netID].deg -3;
			treeedges = sttrees[netID].edges;
			for(edgeID=0; edgeID<numEdges; edgeID++)
			{
				if(sttrees[netID].edges[edgeID].len>0)
				{
					gridsX = sttrees[netID].edges[edgeID].route.gridsX;
					gridsY = sttrees[netID].edges[edgeID].route.gridsY;
					for(i=0; i<sttrees[netID].edges[edgeID].route.routelen; i++)
					{
						if(gridsX[i]==gridsX[i+1]) // a vertical edge
						{
							min_y = min(gridsY[i], gridsY[i+1]);
							v_edges[min_y*xGrid+gridsX[i]].usage += 1;
						}
						else ///if(gridsY[i]==gridsY[i+1])// a horizontal edge
						{
							min_x = min(gridsX[i], gridsX[i+1]);
							h_edges[gridsY[i]*(xGrid-1)+min_x].usage += 1;
						}
					}
				}
			}
		}

	}
}

void freeRR (void)
{
	int i,j,netID,p,q, k, edgeID, numEdges, xmin, xmax, ymin, ymax, numNodes;
	int n1, n2, x1,x2,y1,y2;

	TreeEdge *treeedges, *treeedge;
    TreeNode *treenodes;

	if (sttreesBK != NULL) {
		for(netID=0; netID<numValidNets; netID++) {

			numEdges = 2 * sttreesBK[netID].deg -3;
			treeedges = sttreesBK[netID].edges;
			for(edgeID=0; edgeID<numEdges; edgeID++)
			{
				if(sttreesBK[netID].edges[edgeID].len>0)
				{
					free (sttreesBK[netID].edges[edgeID].route.gridsX);
					free (sttreesBK[netID].edges[edgeID].route.gridsY);
				}
			}
			free(sttreesBK[netID].nodes);
			free(sttreesBK[netID].edges);
		}
		free(sttreesBK);
	}
}



} //namespace sproute


