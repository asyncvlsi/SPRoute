#ifndef GLOBAL_H
#define GLOBAL_H

#include "defDataBase.h"
#include "lefDataBase.h"


parser::defDataBase defDB;
   
parser::lefDataBase lefDB;

#define GLOBAL_CAP_REDUCTION 4
#define CAP_ADJ_FACTOR 0.8

int GLOBAL_CAP_ADJ(int x)
{
	if(x == 0)
		return 0;
	else
		return ((float) x * CAP_ADJ_FACTOR < 2)?  1 : x * CAP_ADJ_FACTOR - 1;
}

#define MAX_FLOW 0

//#define OPENDB 

#endif
