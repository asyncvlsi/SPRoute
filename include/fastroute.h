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
//#include "maze3D.h"
#include "maze_finegrain.h"
#include "maze_finegrain_lateupdate.h"
#include "maze_lock.h"

#include "algo.h"
#include "grGen.h"
#include "detpart.h"
#include "a_star.h"
#include "detpart_astar.h"
#include "detpart_astar_data.h"
#include "detpart_astar_local.h"



void printUndone(galois::LargeArray<bool>& done) {
	cout << "undone nets: " << endl;
	for(int netID=0; netID<numValidNets; netID++) {
		if(!done[netID])
			cout << netID << " " << nets[netID]->name << endl;
	}
}





#endif
