#include "defDataBase.h"

namespace sproute_db
{

Gcell& defDataBase::getGcell(int x, int y, int z)
{
    int loc = z * (gcellGridDim.x * gcellGridDim.y) + y * gcellGridDim.x + x;
    return gcells.at(loc);
}

int find_Gcell(int pin_in, std::vector<int> GcellBoundaries)
{
	auto it = std::upper_bound(GcellBoundaries.begin(), GcellBoundaries.end(), pin_in);
	int x = std::distance(GcellBoundaries.begin(), it) - 1;

	if(x == -1)
		x++;
	else if(it == GcellBoundaries.end())
		x--;

	return x;
}

}//namespace sproute_db



