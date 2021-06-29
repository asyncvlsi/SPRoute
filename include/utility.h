#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <stdio.h>
#include <stdlib.h>

#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "global_variable_extern.h"

namespace sproute {

void printEdge(int netID, int edgeID);
void plotTree(int netID);
void getlen();
void ConvertToFull3DType2 ();

static int comparePVMINX (const void *a, const void *b)
{
	if (((OrderNetPin*)a)->minX > ((OrderNetPin*)b)->minX) return 1;
	else if (((OrderNetPin*)a)->minX == ((OrderNetPin*)b)->minX) return 0;
    else return -1;
}

static int comparePVPV (const void *a, const void *b)
{
	if (((OrderNetPin*)a)->npv > ((OrderNetPin*)b)->npv) return 1;
    else if (((OrderNetPin*)a)->npv == ((OrderNetPin*)b)->npv) return 0;
    else return -1;
}

void netpinOrderInc();
void fillVIA();
int threeDVIA();
void assignEdge(int netID, int edgeID, bool processDIR);
void newLayerAssignmentV4();
int findLayer(int netID, TreeNode treenode);
void newLA ();
void printEdge3D(int netID, int edgeID);

void printTree3D(int netID);
void checkRoute3D();
void write3D();

static int compareTEL (const void *a, const void *b)
{
	if (((OrderTree*)a)->xmin < ((OrderTree*)b)->xmin) return 1;
    else if (((OrderTree*)a)->xmin == ((OrderTree*)b)->xmin) {
		if(a < b)	
			return 1;
		else if(a > b)
			return -1;
		else {
			cout << "same pointer: error!" << endl;
			exit(1);
		}	
	} //TODO: use this after fixing bug of ispd19_test8_metal5
    else return -1;
}

void StNetOrder();
void recoverEdge(int netID, int edgeID);
void checkUsage();

static int compareEdgeLen (const void *a, const void *b)
{
	if (((OrderNetEdge*)a)->length < ((OrderNetEdge*)b)->length) return 1;
    else if (((OrderNetEdge*)a)->length == ((OrderNetEdge*)b)->length) {
		if(a < b)
			return 1;
		else if(a > b)
			return -1;
		else {
			cout << "ERROR: two same edge pointer! " << endl;
			exit(1);
		}
		
	}
    else return -1;
}



void netedgeOrderDec(int netID, OrderNetEdge* netEO);
void printEdge2D(int netID, int edgeID);
void printTree2D(int netID);
bool checkRoute2DTree(int netID);


struct TD{
	int id;
	float cost;
};

struct BBox{
	int xmin;
	int ymin;
	int xmax;
	int ymax;
	int hSpan;
	int vSpan;
};//lower_left corner and upper_right corner



struct wire
{
    int x1,y1,x2,y2;
    int netID;
};


static int ordercost(const void *a,  const void *b)
{
    struct TD *pa, *pb;
    
    pa = *(struct TD**)a;
    pb = *(struct TD**)b;
    
    if (pa->cost < pb->cost) return 1;
    if (pa->cost > pb->cost) return -1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);
}//decreasing order

static int ordervSpan(const void *a,  const void *b)
{
    struct BBox *pa, *pb;
    
    pa = *(struct BBox**)a;
    pb = *(struct BBox**)b;
    
    if (pa->vSpan < pb->vSpan) return -1;
    if (pa->vSpan > pb->vSpan) return 1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);
}

static int orderhSpan(const void *a,  const void *b)
{
    struct BBox *pa, *pb;
    
    pa = *(struct BBox**)a;
    pb = *(struct BBox**)b;
    
    if (pa->hSpan < pb->hSpan) return -1;
    if (pa->hSpan > pb->hSpan) return 1;
    return 0;
   // return ((struct Segment*)a->x1-(struct Segment*)b->x1);
}

// binary search to map the new coordinates to original coordinates



	
// Copy Routing Solution for the best routing solution so far
void copyRS (void);
void copyBR ();
void freeRR (void);

} //namespace sproute


#endif
