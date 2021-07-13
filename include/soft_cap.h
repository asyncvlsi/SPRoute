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

int GLOBAL_CAP_ADJ(int x, float rudy, int layerID); 

#endif
