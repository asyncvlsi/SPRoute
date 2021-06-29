#ifndef SOFT_CAP_H
#define SOFT_CAP_H

#define OBS_NO_STOP 0 // 1 == go through OBS, 0 == hard stop

#define GLOBAL_CAP_REDUCTION 4
#define CAP_ADJ_FACTOR 1.0

#define M2_ADJ_MIN 0.45
#define M2_ADJ_MAX 0.9
#define M2_ADJ_MID 3.5
#define M2_ADJ_K 2.0

#define M3_ADJ_MIN 0.45
#define M3_ADJ_MAX 0.9
#define M3_ADJ_MID 3.5
#define M3_ADJ_K 2.0

#define MID_ADJ_MIN 0.9
#define MID_ADJ_MAX 0.9
#define MID_ADJ_MID 5.0
#define MID_ADJ_K 2.0

#define HIGH_ADJ 0.7

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


#endif
