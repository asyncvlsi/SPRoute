#ifndef _EDGESHIFT_H_
#define _EDGESHIFT_H_

#include <stdio.h>
#include <stdlib.h>
#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "route.h"
#include "ripup.h"

namespace sproute {

int edgeShift(Tree* t);

// exchange Steiner nodes at the same position, then call edgeShift()
int edgeShiftNew(Tree* t);

} //namespace sproute
#endif
