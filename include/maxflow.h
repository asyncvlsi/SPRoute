#ifndef MAXFLOW_H
#define MAXFLOW_H
#include <queue>
#include "galois/Galois.h"
#include "galois/Reduction.h"
#include "galois/PriorityQueue.h"
#include "galois/Timer.h"
#include "galois/graphs/TypeTraits.h"
#include "galois/substrate/SimpleLock.h"
#include "galois/AtomicHelpers.h"
#include "galois/runtime/Profile.h"

#include "galois/graphs/Graph.h"

using namespace parser;

#define FLOW_ALPHA 6
#define FLOW_BETA 12

struct FlowNode {
	int x;
	int y;
	int z;
	int id;
	bool blocked;
	int64_t excess;
	int height;
	int current;

	FlowNode() : x(0), y(0), z(0), id(-1), blocked(false), excess(0), height(1), current(0) {}
};

struct WorkingGcell
{
	int x;
	int y;
	int z;
	WorkingGcell() : x(0), y(0), z(0) {}
	WorkingGcell(int xx, int yy, int zz) :  x(xx), y(yy), z(zz) {}
};

std::ostream& operator<< (std::ostream& os, const FlowNode& n) {
  os << "("
     << ", excess: " << n.excess << ", height: " << n.height
     << ", current: " << n.current << ")";
  return os;
}

class GraphTemplate
{
public:
	std::map< Point3D<int>, int> point2nodeidx;
	std::vector<pair<int, int>> edgelist;

	GraphTemplate(): point2nodeidx(), edgelist() {}

};

Range<int> getTrackRange(int layerIdx, Rect2D<float> rect, bool expand)
{
    int lower, upper, startTrackIdx, endTrackIdx;
    int trackStart, trackStep;
    string direction = lefDB.layers.at(layerIdx).direction;
    string layerName = lefDB.layers.at(layerIdx).name;

    if(defDB.layerName2trackidx.count(layerName) == 0)
    {
        cout << "unable to find layer in layer tracks: " << layerName << endl;
        exit(1);
    }

    if(direction == "HORIZONTAL") // for pessimism, disable one more track 
    {
        lower = rect.lowerLeft.y;
        upper = rect.upperRight.y;  
    }
    else if(direction == "VERTICAL")
    {
        lower = rect.lowerLeft.x;
        upper = rect.upperRight.x;
    }
    else
    {
        cout << "unkown direction of layer : " << layerIdx << endl;
        exit(1);
    }

    int trackIdx = defDB.layerName2trackidx.find(layerName)->second;
    trackStart = defDB.tracks.at(trackIdx).start;
    trackStep = defDB.tracks.at(trackIdx).step;

    if(expand)
    {
	    startTrackIdx = (lower - trackStep - trackStart) / trackStep;
	    startTrackIdx = (startTrackIdx < 0)? 0 : startTrackIdx;
	    endTrackIdx = (upper + trackStep - trackStart) / trackStep;
	}
	else
	{
		startTrackIdx = (lower - trackStart) / trackStep + 1;
	    endTrackIdx = (upper - trackStart) / trackStep;
	}

    return Range<int>(startTrackIdx, endTrackIdx);
}

Range<int> getTrackRange(int layerIdx, Rect2D<int> rect, bool expand)
{
    int lower, upper, startTrackIdx, endTrackIdx;
    int trackStart, trackStep;
    string direction = lefDB.layers.at(layerIdx).direction;
    string layerName = lefDB.layers.at(layerIdx).name;

    if(defDB.layerName2trackidx.count(layerName) == 0)
    {
        cout << "unable to find layer in layer tracks: " << layerName << endl;
        exit(1);
    }

    if(direction == "HORIZONTAL") // for pessimism, disable one more track 
    {
        lower = rect.lowerLeft.y;
        upper = rect.upperRight.y;
    }
    else if(direction == "VERTICAL")
    {
        lower = rect.lowerLeft.x;
        upper = rect.upperRight.x;
    }
    else
    {
        cout << "unkown direction of layer : " << layerIdx << endl;
        exit(1);
    }

    int trackIdx = defDB.layerName2trackidx.find(layerName)->second;
    trackStart = defDB.tracks.at(trackIdx).start;
    trackStep = defDB.tracks.at(trackIdx).step;

    if(expand)
    {
	    startTrackIdx = (lower - trackStep - trackStart) / trackStep;
	    startTrackIdx = (startTrackIdx < 0)? 0 : startTrackIdx;
	    endTrackIdx = (upper + trackStep - trackStart) / trackStep;
	}
	else
	{
		startTrackIdx = (lower - trackStart) / trackStep + 1;
	    endTrackIdx = (upper - trackStart) / trackStep;
	}

    return Range<int>(startTrackIdx, endTrackIdx);
}



#if MAX_FLOW

using FlowGraph = galois::graphs::First_SepInOut_Graph<FlowNode, int32_t, true>;
//using FlowGraph = galois::graphs::LC_InOut_Graph<FlowNode, int32_t, true>;
using FlowGNode = FlowGraph::GraphNode;

bool coverByOBS(Point2D<int> p, Rect2D<int> rect, int expand)
{
	Rect2D<int> ExpandRect;
	ExpandRect.lowerLeft.x = rect.lowerLeft.x - expand;
	ExpandRect.lowerLeft.y = rect.lowerLeft.y - expand;

	ExpandRect.upperRight.x = rect.upperRight.x + expand;
	ExpandRect.upperRight.y = rect.upperRight.y + expand;

	return ExpandRect.cover(p);
}

void localizeNode(Point3D<int>& p, Rect2D<int> gcellRect)
{
	p.x = p.x - gcellRect.lowerLeft.x;
	p.y = p.y - gcellRect.lowerLeft.y;
}

void globalizeNode(Point2D<int>& p, Rect2D<int> gcellRect)
{
	p.x = p.x + gcellRect.lowerLeft.x;
	p.y = p.y + gcellRect.lowerLeft.y;
}


void constructGraph(FlowGraph& graph, GraphTemplate& graphTemplate, int layer, Rect2D<int> GcellRect,
	std::vector<Rect2D<int>>& curOBSlist, std::vector<Rect2D<int>>& botOBSlist, std::vector<Rect2D<int>>& topOBSlist)
{
	int expand = 0;
	string layerName = lefDB.layers.at(layer * 2).name;
	int trackIdx = defDB.layerName2trackidx.find(layerName)->second;
	int trackStep = defDB.tracks.at(trackIdx).step;
	expand = std::max(expand, trackStep);

	if(layer < defDB.gcellGridDim.z - 1)
	{
		string topLayerName = lefDB.layers.at((layer + 1) * 2).name;
		int topTrackIdx = defDB.layerName2trackidx.find(topLayerName)->second;
		int topTrackStep = defDB.tracks.at(topTrackIdx).step;
		expand = std::max(expand, topTrackStep);
	}

	std::vector<pair<int, int>>& edgelist = graphTemplate.edgelist;
    std::map< Point3D<int>, int>& point2nodeidx = graphTemplate.point2nodeidx;
    std::map<int, FlowGNode> localGraphMap;

	FlowNode srcData;
	srcData.x = srcData.y = srcData.z = -1;
	srcData.id = 0;
	auto src = graph.createNode(srcData); //by default, graph.begin() == src.
	graph.addNode(src);
	localGraphMap.insert(pair<int, FlowGNode>(0, src));

	FlowNode dstData;
	dstData.x = dstData.y = dstData.z = -2;
	dstData.id = 1;
	auto dst = graph.createNode(dstData); //by default, graph.begin() + 1 == dst.
	graph.addNode(dst);
	localGraphMap.insert(pair<int, FlowGNode>(1, dst));

	for(auto p : point2nodeidx)
	{
		FlowNode tmpData;
		tmpData.id = graph.size();
		auto gnode = graph.createNode(tmpData);
		graph.addNode(gnode);
		localGraphMap.insert(pair<int, FlowGNode>(tmpData.id, gnode));
	}

	for(auto p : point2nodeidx)
	{
		auto point = p.first;
		auto nodeid = p.second;
		auto Iter = localGraphMap.find(nodeid)->second;
		auto& nodeData = graph.getData(Iter);
		nodeData.x = p.first.x;
		nodeData.y = p.first.y;
		nodeData.z = p.first.z;


		Point2D<int> tmpPoint;
		tmpPoint.x = point.x;
		tmpPoint.y = point.y;
		globalizeNode(tmpPoint, GcellRect);
		
		bool blocked = false;
		if(point.z == layer)
		{
			for(auto& obs : curOBSlist)
			{
				if(coverByOBS(tmpPoint, obs, expand))
				{
					blocked = true;
					break;
				}
			}
		}
		else if(point.z == layer - 1)
		{
			for(auto& obs : botOBSlist)
			{
				if(coverByOBS(tmpPoint, obs, expand))
				{
					blocked = true;
					break;
				}
			}
		}
		else if(point.z == layer + 1)
		{
			for(auto& obs : topOBSlist)
			{
				if(coverByOBS(tmpPoint, obs, expand))
				{
					blocked = true;
					break;
				}
			}
		}
		else
		{
			cout << "unable to find point: " << point.x << " " << point.y << " " << point.z << " layer: " << layer << endl;
			exit(1);
		}

		nodeData.blocked = blocked;
		//cout << " " << (point.x - 200) / 400 << " " << (point.y - 190)/ 380 << " " << point.z << " block: " << blocked << endl;
	}

	for(auto edge : edgelist)
	{
		int srcid = edge.first;
		int dstid = edge.second;
		auto srcIter = localGraphMap.find(srcid)->second;
		auto srcData = graph.getData(srcIter);

		auto dstIter = localGraphMap.find(dstid)->second;
		auto dstData = graph.getData(dstIter);

		auto e1 = graph.addEdge(dstIter, srcIter); 
		auto e2 = graph.addEdge(srcIter, dstIter);
		if(srcData.blocked || dstData.blocked)
		{
			graph.getEdgeData(e1) = 0;
			graph.getEdgeData(e2) = 0;
		}
		else
		{
			graph.getEdgeData(e1) = 1;
			graph.getEdgeData(e2) = 1;
		}

		//cout << " edge added: " << (srcData.x - 200)/400 << "," << (srcData.y - 190)/380 << "," << srcData.z;
		//cout << " <-> " << (dstData.x - 200)/400 << "," << (dstData.y - 190)/380 << "," << dstData.z << endl;
	}
}

void genLayerGraphTemplate(FlowGraph& graph, GraphTemplate& graphTemplate, 
	std::set<int>& trackOffset, std::set<int>& trackNodeOffset, Rect2D<int> gcellRect, int layer, int midBoundary, string whichLayer, bool print = false)
{
	std::map< Point3D<int>, int>& point2nodeidx = graphTemplate.point2nodeidx;
	std::vector<pair<int, int>>& edgelist = graphTemplate.edgelist;
	
	bool dir_H;
	if(lefDB.layers.at(layer * 2).direction == "HORIZONTAL")
	{
		dir_H = true;
	}
	else if(lefDB.layers.at(layer * 2).direction == "VERTICAL")
	{
		dir_H = false;
	}
	else{
        cout << "unknown layer direction: " << lefDB.layers.at(layer * 2).name << endl;
        exit(1);
    }

    FlowGNode src, dst;
    if(whichLayer == "cur")
    {
    	FlowNode srcData;
    	srcData.x = srcData.y = srcData.z = -1;
    	srcData.id = 0;
		src = graph.createNode(srcData); //by default, graph.begin() == src.
		graph.addNode(src);
		FlowNode dstData;
		dstData.x = dstData.y = dstData.z = -2;
		dstData.id = 1;
		dst = graph.createNode(dstData); //by default, graph.begin() + 1 == dst.
		graph.addNode(dst);
    }
    else if(whichLayer == "top" || whichLayer == "bot")
    {
    	src = *graph.begin();
    	auto ii = graph.begin();
    	std::advance(ii, 1);
    	dst = *ii;
    }
    else
    {
    	cout << "impossible layer in flow graph generation" << endl;
    	exit(1);
    }

    // connect node on its own track
	for(int trackLoc : trackOffset)
	{
		int prevNodeOffset = -1;
		for(auto trackNodeLoc : trackNodeOffset)
		{
			Point3D<int> nodeLoc;
			nodeLoc.z = layer;
			if(dir_H)
			{
				nodeLoc.x = trackNodeLoc;
				nodeLoc.y = trackLoc;
			}
			else
			{
				nodeLoc.x = trackLoc;
				nodeLoc.y = trackNodeLoc;
			}
			localizeNode(nodeLoc, gcellRect);

			if(1)
			{
				FlowNode nodeData;
				nodeData.id = graph.size();
				auto n = graph.createNode(nodeData);
				graph.addNode(n); 
				point2nodeidx.insert(pair<Point3D<int>, int>(nodeLoc, nodeData.id));

				if(prevNodeOffset != -1) // has a node on the left
				{
					Point3D<int> prevLoc;
					prevLoc.z = layer;
					if(dir_H)
					{
						prevLoc.x = prevNodeOffset;
						prevLoc.y = trackLoc;
					}
					else
					{
						prevLoc.x = trackLoc;
						prevLoc.y = prevNodeOffset;
					}
					localizeNode(prevLoc, gcellRect);
					auto previd = point2nodeidx.find(prevLoc)->second;
					assert(point2nodeidx.count(prevLoc) > 0);
					edgelist.push_back(pair<int, int>(nodeData.id, previd));
					if(print)
					{
						cout << "edge added: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z;
						cout << " <-> " << (prevLoc.x - 200) / 400 << "," << (prevLoc.y - 190) / 380 << "," << prevLoc.z << endl;
					}
				}

				//=======================connect node to adj layers============
				if(whichLayer == "bot")
				{
					Point3D<int> curLoc;
					curLoc.z = layer + 1;
					curLoc.x = nodeLoc.x;
					curLoc.y = nodeLoc.y;
					if(point2nodeidx.count(curLoc) > 0)
					{
						auto curid = point2nodeidx.find(curLoc)->second;
						assert(point2nodeidx.count(curLoc) > 0);
						edgelist.push_back(pair<int, int>(nodeData.id, curid));
						if(print)
						{
							cout << "edge added: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z;
							cout << " <-> " << (curLoc.x - 200) / 400 << "," << (curLoc.y - 190) / 380 << "," << curLoc.z << endl;
						}
					}
				}
				else if(whichLayer == "top")
				{
					Point3D<int> curLoc;
					curLoc.z = layer - 1;
					curLoc.x = nodeLoc.x;
					curLoc.y = nodeLoc.y;
					if(point2nodeidx.count(curLoc) > 0)
					{
						auto curid = point2nodeidx.find(curLoc)->second;
						assert(point2nodeidx.count(curLoc) > 0);
						edgelist.push_back(pair<int, int>(nodeData.id, curid));
						if(print)
						{
							cout << "edge added: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z;
							cout << " <-> " << (curLoc.x - 200) / 400 << "," << (curLoc.y - 190) / 380 << "," << curLoc.z << endl;
						}
					}
				}


				//==================connect node to src & dst======================
				if(whichLayer =="cur")
				{
					int leftMost, rightMost;
					if(trackNodeLoc == *(trackNodeOffset.begin())) // is the left most node
					{
						edgelist.push_back(pair<int, int>(nodeData.id, 0));
						if(print)
							cout << "edge added to src: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z << endl;
					}
					else if(trackNodeLoc == *(trackNodeOffset.rbegin())) // is the right most node
					{
						edgelist.push_back(pair<int, int>(nodeData.id, 1));
						if(print)
							cout << "edge added to dst: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z << endl;
					}
				}
				else if(whichLayer == "bot" || whichLayer == "top")
				{
					if(trackNodeLoc == *(trackNodeOffset.rbegin()) || trackNodeLoc == *(trackNodeOffset.begin())) // left most or right most node
					{
						if(trackLoc < midBoundary) //src side
				 		{
				 			edgelist.push_back(pair<int, int>(nodeData.id, 0));
				 			if(print)
				 				cout << "edge added to src: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z << endl;
				 		}
						else //dst side
						{
							edgelist.push_back(pair<int, int>(nodeData.id, 1));
							if(print)
								cout << "edge added to dst: " << (nodeLoc.x - 200) / 400 << "," << (nodeLoc.y - 190) / 380 << "," << nodeLoc.z << endl;
						}
					}
				}

				prevNodeOffset = trackNodeLoc;
			}

		}//iterate on track node offset
	}//iterate on track
}




int genGraphTemplateGcell(int x, int y, int z, GraphTemplate& graphTemplate, bool print = false)
{
    int next_x, next_y;
    Gcell gcell = defDB.getGcell(x, y, z);
    int midBoundary;
    if(lefDB.layers.at(z * 2).direction == "HORIZONTAL")
    {
        next_x = x + 1;
        next_y = y;
        midBoundary = defDB.xGcellBoundaries.at(x + 1);
    }
    else if(lefDB.layers.at(z * 2).direction == "VERTICAL")
    {
        next_x = x;
        next_y = y + 1;
        midBoundary = defDB.yGcellBoundaries.at(y + 1);
    }
    else{
        
        cout << "unknown layer direction: " << lefDB.layers.at(z * 2).name << endl;
        exit(1);
    }

    Gcell nextGcell = defDB.getGcell(next_x, next_y, z);

    if(gcell.obs_state == FULLYCOVERED || nextGcell.obs_state == FULLYCOVERED)
        return 0;

    Point2D<int> GcellLowerLeft(defDB.xGcellBoundaries.at(x), defDB.yGcellBoundaries.at(y));
    Point2D<int> GcellUpperRight(defDB.xGcellBoundaries.at(x + 1), defDB.yGcellBoundaries.at(y + 1));
    Rect2D<int> GcellRect(GcellLowerLeft, GcellUpperRight);

    Point2D<int> nextGcellUpperRight(defDB.xGcellBoundaries.at(next_x + 1), defDB.yGcellBoundaries.at(next_y + 1));
    Rect2D<int> GraphRegion(GcellLowerLeft, nextGcellUpperRight);

//===============================================================
    std::set<int> curTrackOffset, trackNodeOffset, botTrackOffset, topTrackOffset; 
    std::set<int> curOBSlist, botOBSlist, topOBSlist;

    Range<int> curLayerTrackRange = getTrackRange(z*2, GraphRegion, false);
    string curLayerName = lefDB.layers.at(2*z).name;
    int curTrackIdx = defDB.layerName2trackidx.find(curLayerName)->second;
    Track curTrack = defDB.tracks.at(curTrackIdx);
    int curTrackStart = curTrack.start;
    int curTrackStep = curTrack.step;
    for(int i = curLayerTrackRange.start; i <= curLayerTrackRange.end; i++)
        curTrackOffset.insert(curTrackStart + i * curTrackStep);

//================================================================
    //add track offset of bot layer

    if(z - 1 > 0)
    {
        Range<int> botLayerTrackRange = getTrackRange((z - 1)*2, GraphRegion, false);

        string botLayerName = lefDB.layers.at(2*(z - 1)).name;
        int botTrackIdx = defDB.layerName2trackidx.find(botLayerName)->second;
        Track botTrack = defDB.tracks.at(botTrackIdx);
        int botTrackStart = botTrack.start;
        int botTrackStep = botTrack.step;
        for(int i = botLayerTrackRange.start; i <= botLayerTrackRange.end; i++)
        {
            trackNodeOffset.insert(botTrackStart + i * botTrackStep);
            botTrackOffset.insert(botTrackStart + i * botTrackStep);
        }
    }
//================================================================
    //add track offset of top layer

    if(z + 1 < defDB.gcellGridDim.z)
    {
        Range<int> topLayerTrackRange = getTrackRange((z + 1)*2, GraphRegion, false);

        string topLayerName = lefDB.layers.at(2*(z + 1)).name;
        int topTrackIdx = defDB.layerName2trackidx.find(topLayerName)->second;
        Track topTrack = defDB.tracks.at(topTrackIdx);
        int topTrackStart = topTrack.start;
        int topTrackStep = topTrack.step;
        for(int i = topLayerTrackRange.start; i <= topLayerTrackRange.end; i++)
        {
            trackNodeOffset.insert(topTrackStart + i * topTrackStep);
            topTrackOffset.insert(topTrackStart + i * topTrackStep);
        }
    }
    FlowGraph graph;
    //genFlowGraph
    genLayerGraphTemplate(graph, graphTemplate, curTrackOffset, trackNodeOffset, GcellRect, z, midBoundary, string("cur"), print);
    if(z - 1 > 0)
    {
        genLayerGraphTemplate(graph, graphTemplate, botTrackOffset, curTrackOffset, GcellRect, z - 1, midBoundary,  string("bot"), print);
    }
    if(z + 1 < defDB.gcellGridDim.z)
    {
        genLayerGraphTemplate(graph, graphTemplate, topTrackOffset, curTrackOffset, GcellRect, z + 1, midBoundary,  string("top"), print);
    }

}

void genGraphTemplate(GraphTemplate* graphTemplates)
{
	for(int layer = 1; layer < defDB.gcellGridDim.z; layer++)
	{
		genGraphTemplateGcell(2, 2, layer, graphTemplates[ layer * 2 * 2 ]);
		genGraphTemplateGcell(3, 2, layer, graphTemplates[ layer * 2 * 2  + 1]);
		genGraphTemplateGcell(2, 3, layer, graphTemplates[ layer * 2 * 2  + 2]);
		genGraphTemplateGcell(3, 3, layer, graphTemplates[ layer * 2 * 2  + 3]);

	}
}

void reduceCapacity(FlowGraph& graph, const FlowGNode& src, const FlowGNode& dst, int64_t amount) {
	auto e1 = graph.findEdge(src, dst, galois::MethodFlag::UNPROTECTED);
	auto e2 = graph.findEdge(dst, src, galois::MethodFlag::UNPROTECTED);
    FlowGraph::edge_data_type& cap1 = graph.getEdgeData(e1, galois::MethodFlag::UNPROTECTED);
    FlowGraph::edge_data_type& cap2 = graph.getEdgeData(e2, galois::MethodFlag::UNPROTECTED);
    cap1 -= amount;
    cap2 += amount;
}

void initializeMaxFlow(FlowGraph& graph, std::queue<FlowGNode>& initial, int numEdges)
{
	auto source = *graph.begin();
	graph.getData(source).height = graph.size();
    /*reverseDirectionEdgeIterator.allocateLocal(numEdges);
    // memoize the reverse direction edge-iterators
    for(auto n : graph)
    {
         for (auto ii : graph.edges(n, galois::MethodFlag::UNPROTECTED)) {
           FlowGNode dst = graph.getEdgeDst(ii);
           reverseDirectionEdgeIterator[*ii] = graph.findEdge(dst, n, galois::MethodFlag::UNPROTECTED);
         }
    }*/

    int init_cnt = 0;
    int push_cnt = 0;

    for (auto ii : graph.edges(source)) {
		FlowGNode dst   = graph.getEdgeDst(ii);
		int cap = graph.getEdgeData(ii);
		reduceCapacity(graph, source, dst, cap);
		FlowNode& node = graph.getData(dst, galois::MethodFlag::UNPROTECTED);
		node.excess += cap;
		//if(node.z == 1)
		//	cout << (node.x - 200) / 400<< " " << (node.y - 190) / 380<< " " << node.z << endl;
		init_cnt++;
		if (cap > 0)
		{
			initial.push(dst);
			push_cnt++;
		}
    }
    //cout << " initcnt : " << init_cnt << " pushcnt: " << push_cnt << endl;
}

void relabelFlow(FlowGraph& graph, const FlowGNode& src) {
    int minHeight = std::numeric_limits<int>::max();
    int minEdge   = 0;

    int current = 0;
    for (auto ii : graph.edges(src, galois::MethodFlag::UNPROTECTED)) {
      FlowGNode dst   = graph.getEdgeDst(ii);
      int64_t cap = graph.getEdgeData(ii);
      if (cap > 0) {
        const FlowNode& dnode = graph.getData(dst, galois::MethodFlag::UNPROTECTED);
        if (dnode.height < minHeight) {
          minHeight = dnode.height;
          minEdge   = current;
        }
      }
      ++current;
    }

    assert(minHeight != std::numeric_limits<int>::max());
    ++minHeight;

    FlowNode& node = graph.getData(src, galois::MethodFlag::UNPROTECTED);
    if (minHeight < (int)graph.size()) {
      node.height  = minHeight;
      node.current = minEdge;
    } else {
      node.height = graph.size();
    }
  }

bool dischargeFlow(FlowGraph& graph, const FlowGNode& src, std::queue<FlowGNode>& initial, const FlowGNode& source, const FlowGNode& sink) {

    FlowNode& srcData = graph.getData(src, galois::MethodFlag::UNPROTECTED);

    bool relabeled = false;

    if (srcData.excess == 0 || srcData.height >= (int)graph.size()) {
      return false;
    }

    while (true) {
      // galois::MethodFlag flag = relabeled ? galois::MethodFlag::UNPROTECTED :
      // galois::MethodFlag::WRITE;
      galois::MethodFlag flag = galois::MethodFlag::UNPROTECTED;
      bool finished           = false;
      int current             = srcData.current;

      auto ii = graph.edge_begin(src, flag);
      auto ee = graph.edge_end(src, flag);

      std::advance(ii, srcData.current);

      for (; ii != ee; ++ii, ++current) {
        FlowGNode dst   = graph.getEdgeDst(ii);
        int64_t cap = graph.getEdgeData(ii);
        if (cap == 0) // || current < node.current)
          continue;

        FlowNode& dstData = graph.getData(dst, galois::MethodFlag::UNPROTECTED);
        if (srcData.height - 1 != dstData.height)
          continue;

        // Push flow
        int64_t amount = std::min(srcData.excess, cap);
        reduceCapacity(graph, src, dst, amount);

        // Only add once
        if (dst != sink && dst != source && dstData.excess == 0)
          initial.push(dst);

        assert(srcData.excess >= amount);
        srcData.excess -= amount;
        dstData.excess += amount;

        if (srcData.excess == 0) {
          finished     = true;
          srcData.current = current;
          break;
        }
      }

      if (finished)
        break;

      relabelFlow(graph, src);
      relabeled = true;

      if (srcData.height == (int)graph.size())
        break;

      // prevHeight = node.height;
    }

    return relabeled;
  }



void detDischarge(FlowGraph& graph, std::queue<FlowGNode>& initial, int& counter, bool& should_global_relabel, int relabel_interval,
	const FlowGNode& source, const FlowGNode& sink)
{
	while(!initial.empty())
	{
		FlowGNode src = initial.front();
		initial.pop();

		int increment = 1;
		if(dischargeFlow(graph, src, initial, source, sink)) {
			increment += FLOW_BETA;
		}

		counter += increment;
		if(counter >= relabel_interval)
		{
			should_global_relabel = true;
			break;
		}
    }

}

  /**
   * Do reverse BFS on residual graph.
   */
void updateHeights(FlowGraph& graph, const FlowGNode& source, const FlowGNode& sink) {

	std::queue<FlowGNode> WL;
	WL.push(sink);

	while(!WL.empty())
	{
		FlowGNode src = WL.front();
		WL.pop();
		for (auto ii : graph.edges(src, galois::MethodFlag::UNPROTECTED)) {
			FlowGNode dst = graph.getEdgeDst(ii);
			auto redgeIt = graph.findEdge(dst, src, galois::MethodFlag::UNPROTECTED);
			int rdata = graph.getEdgeData(redgeIt);
			if (rdata > 0) {
				FlowNode& node = graph.getData(dst, galois::MethodFlag::UNPROTECTED);
				int newHeight = graph.getData(src, galois::MethodFlag::UNPROTECTED).height + 1;
			  
			    if (newHeight < node.height) {
					node.height = newHeight;
					WL.push(dst);
			    }
			}
		} // end for
    }
}

void globalRelabel(FlowGraph& graph, std::queue<FlowGNode>& incoming, const FlowGNode& source, const FlowGNode& sink) 
{
	while(!incoming.empty()) //clear work list
		incoming.pop();


	for(const auto& src : graph)
	{               
		FlowNode& node = graph.getData(src, galois::MethodFlag::UNPROTECTED);
		node.height  = graph.size();
		node.current = 0;
		if (src == sink)
			node.height = 0;
	}
	updateHeights(graph, source, sink);

	for(const auto& src : graph)
	{
		FlowNode& node = graph.getData(src, galois::MethodFlag::UNPROTECTED);
		if (src == sink || src == source || node.height >= (int)graph.size())
	        continue;
	    if (node.excess > 0)
	        incoming.push(src);
	}
}

int computeMaxFlow(FlowGraph& graph, int numEdges)
{
	std::queue<FlowGNode> initial;
	initializeMaxFlow(graph, initial, numEdges);

	bool should_global_relabel = false;

	int relabel_interval = graph.size() * FLOW_ALPHA + numEdges / 3;

	auto source = *graph.begin();
	auto ii = graph.begin();
    std::advance(ii, 1);
    auto sink = *ii;

	while (!initial.empty()) {
		int counter = 0;

		detDischarge(graph, initial, counter, should_global_relabel, relabel_interval, source, sink);

		if (should_global_relabel) {
			globalRelabel(graph, initial, source, sink);
			should_global_relabel = false;
			//std::cout << " Flow after global relabel: "
			//          << graph.getData(sink).excess << "\n";
		} else {
			break;
		}
    }

    return graph.getData(sink).excess;

}

#endif

/*void genCurLayerGraph(FlowGraph& graph, std::map< Point3D<int>, FlowGNode>& point2nodeidx, 
	std::set<int>& trackOffset, std::set<int>& trackNodeOffset, std::set<int>& OBSlist, int layer, int gcellBoundary, string whichLayer)
{
	bool dir_H;
	if(lefDB.layers.at(layer * 2).direction == "HORIZONTAL")
		dir_H = true;
	else if(lefDB.layers.at(layer * 2).direction == "VERTICAL")
		dir_H = false;
	else{
        cout << "unknown layer direction: " << lefDB.layers.at(layer * 2).name << endl;
        exit(1);
    }

    FlowGNode src, dst;
    if(whichLayer == "cur")
    {
    	FlowGraphNode srcData;
		src = graph.createNode(srcData); //by default, graph.begin() == src.
		graph.addNode(src);
		FlowGraphNode dstData;
		dst = graph.createNode(dstData); //by default, graph.begin() + 1 == dst.
		graph.addNode(dst);
    }
    else if(whichLayer == "top" || whichLayer == "bot")
    {
    	src = *graph.begin();
    	auto ii = graph.begin();
    	std::advance(ii, 1);
    	dst = *ii;
    }
    else
    {
    	cout << "impossible layer in flow graph generation" << endl;
    	exit(1);
    }

    // connect node on its own track
	for(int trackLoc : trackOffset)
	{
		int prevNodeOffset = -1;
		for(auto trackNodeLoc : trackNodeOffset)
		{
			Point3D<int> nodeLoc;
			nodeLoc.z = layer;
			if(dir_H)
			{
				nodeLoc.x = trackNodeLoc;
				nodeLoc.y = trackLoc;
			}
			else
			{
				nodeLoc.x = trackLoc;
				nodeLoc.y = trackNodeLoc;
			}

			if(!coverByOBS(nodeLoc.x, nodeLoc.y, OBSlist, layer))
			{
				FlowGraphNode nodeData;
				auto n = graph.createNode(nodeData);
				graph.addNode(n);
				point2nodeidx.insert(pair<Point3D<int>, FlowGNode>(nodeLoc, n));

				if(prevNodeOffset != -1) // has a node on the left
				{
					Point3D<int> prevLoc;
					prevLoc.z = layer;
					if(dir_H)
					{
						prevLoc.x = prevNodeOffset;
						prevLoc.y = trackLoc;
					}
					else
					{
						prevLoc.x = trackLoc;
						prevLoc.y = prevNodeOffset;
					}
					auto prevGNode = point2nodeidx.find(prevLoc)->second;
					auto adj_e1 = graph.addEdge(prevGNode, n, galois::MethodFlag::WRITE); // connect the left most node to src
					auto adj_e2 = graph.addEdge(n, prevGNode, galois::MethodFlag::WRITE);
					graph.getEdgeData(adj_e1, galois::MethodFlag::WRITE) = 1;
					graph.getEdgeData(adj_e2, galois::MethodFlag::WRITE) = 1;
				}

				//=======================connect node to adj layers============
				if(whichLayer == "bot")
				{
					Point3D<int> curLoc;
					curLoc.z = layer + 1;
					curLoc.x = nodeLoc.x;
					curLoc.y = nodeLoc.y;
					if(point2nodeidx.count(curLoc) > 0)
					{
						auto curGNode = point2nodeidx.find(curLoc)->second;
						auto adj_ebot1 = graph.addEdge(n, curGNode, galois::MethodFlag::WRITE); 
						auto adj_ebot2 = graph.addEdge(curGNode, n, galois::MethodFlag::WRITE); 
						graph.getEdgeData(adj_ebot1, galois::MethodFlag::WRITE) = 1;
						graph.getEdgeData(adj_ebot2, galois::MethodFlag::WRITE) = 1;
					}
				}
				else if(whichLayer == "top")
				{
					Point3D<int> curLoc;
					curLoc.z = layer - 1;
					curLoc.x = nodeLoc.x;
					curLoc.y = nodeLoc.y;
					if(point2nodeidx.count(curLoc) > 0)
					{
						auto curGNode = point2nodeidx.find(curLoc)->second;
						auto adj_etop1 = graph.addEdge(n, curGNode, galois::MethodFlag::WRITE); 
						auto adj_etop2 = graph.addEdge(curGNode, n, galois::MethodFlag::WRITE); 
						graph.getEdgeData(adj_etop1, galois::MethodFlag::WRITE) = 1;
						graph.getEdgeData(adj_etop2, galois::MethodFlag::WRITE) = 1;
					}
				}


				//==================connect node to src & dst======================
				if(whichLayer =="cur")
				{
					int leftMost, rightMost;
					if(trackNodeLoc == *(trackNodeOffset.begin())) // is the left most node
					{
						auto adj_src1 = graph.addEdge(src, n, galois::MethodFlag::WRITE); // connect the left most node to src

						auto adj_src2 = graph.addEdge(n, src, galois::MethodFlag::WRITE);
						graph.getEdgeData(adj_src1, galois::MethodFlag::WRITE) = 1;
						graph.getEdgeData(adj_src2, galois::MethodFlag::WRITE) = 1;
					}
					else if(trackNodeLoc == *(trackNodeOffset.rbegin())) // is the right most node
					{
						auto adj_dst1 = graph.addEdge(dst, n, galois::MethodFlag::WRITE); // connect the left most node to src
						auto adj_dst2 = graph.addEdge(n, dst, galois::MethodFlag::WRITE);
						graph.getEdgeData(adj_dst1, galois::MethodFlag::WRITE) = 1;
						graph.getEdgeData(adj_dst2, galois::MethodFlag::WRITE) = 1;
					}
				}
				else if(whichLayer == "bot" || whichLayer == "top")
				{
					if(trackNodeLoc == *(trackNodeOffset.rbegin()) || trackNodeLoc == *(trackNodeOffset.begin())) // left most or right most node
					{
						if(trackLoc < gcellBoundary) //src side
				 		{
				 			auto adj_src1 = graph.addEdge(src, n, galois::MethodFlag::WRITE); // connect the left most node to src
							auto adj_src2 = graph.addEdge(n, src, galois::MethodFlag::WRITE);
							graph.getEdgeData(adj_src1, galois::MethodFlag::WRITE) = 1;
							graph.getEdgeData(adj_src2, galois::MethodFlag::WRITE) = 1;
				 		}
						else //dst side
						{
							auto adj_dst1 = graph.addEdge(dst, n, galois::MethodFlag::WRITE); // connect the left most node to src
							auto adj_dst2 = graph.addEdge(n, dst, galois::MethodFlag::WRITE);
							graph.getEdgeData(adj_dst1, galois::MethodFlag::WRITE) = 1;
							graph.getEdgeData(adj_dst2, galois::MethodFlag::WRITE) = 1;
						}
					}
				}

				prevNodeOffset = trackNodeLoc;
			}
			else
				continue;

		}
	}
}*/

#endif




