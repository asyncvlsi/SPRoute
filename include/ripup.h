#ifndef _RIPUP_H_
#define _RIPUP_H_

#include "DataType.h"
#include "utility.h"

namespace sproute {
// rip-up a L segment
void ripupSegL(Segment* seg);

void ripupSegZ(Segment* seg);

void newRipup(TreeEdge* treeedge, int x1, int y1, int x2, int y2);

bool newRipupType2(TreeEdge* treeedge, TreeNode* treenodes, int x1, int y1,
                   int x2, int y2, int deg);

void printEdgeVEC(TreeEdge* treeedge);

bool newRipupCheckProb(TreeEdge* treeedge, int ripup_threshold, int netID,
                       int edgeID);

bool newRipupCheck(TreeEdge* treeedge, int ripup_threshold, int netID,
                   int edgeID);

bool newRipupCheck_atomic(TreeEdge* treeedge, int ripup_threshold, int netID,
                          int edgeID);

bool newRipupCheck_sort(TreeEdge* treeedge, int ripup_threshold, int netID,
                        int edgeID, bool& is_horizontal, int& grid_pos);

bool newRipupCheck_nosub(TreeEdge* treeedge, int ripup_threshold, int netID,
                         int edgeID); 
bool newRipup3DType3(int netID, int edgeID);

void newRipupNet(int netID);
}// namespace sproute

#endif
