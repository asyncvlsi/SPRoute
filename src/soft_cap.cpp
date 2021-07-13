#include <math.h>
#include "soft_cap.h"

int GLOBAL_CAP_ADJ(int x, float rudy, int layerID) //layerID starting from 0, i.e. 0 = metal1, 1 = metal2, 2 = metal3
{
	if(x == 0)
		return 0;
	else {
		float adj;

		if(layerID == 1) //metal2
			adj = (float) M2_ADJ_MIN + (float)(M2_ADJ_MAX - M2_ADJ_MIN) / (1.0 + exp(M2_ADJ_K * (rudy - M2_ADJ_MID)));
		else if(layerID == 2) //metal3
			adj = (float) M3_ADJ_MIN + (float)(M3_ADJ_MAX - M3_ADJ_MIN) / (1.0 + exp(M3_ADJ_K * (rudy - M3_ADJ_MID)));
		else if(layerID >= 3 && layerID <= 4) //metal4 metal5
			adj = (float) MID_ADJ_MIN + (float)(MID_ADJ_MAX - MID_ADJ_MIN) / (1.0 + exp(MID_ADJ_K * (rudy - MID_ADJ_MID)));
		else if(layerID >= 5) // metal6 and above
			adj = HIGH_ADJ;
		

		//adj = 0.9;

		return ((float) x * adj < 2)?  1 : x * adj;
	}
}