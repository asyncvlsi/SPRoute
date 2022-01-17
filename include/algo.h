#ifndef ALGO_H
#define ALGO_H
#include <iostream>
#include <string>

namespace sproute {

enum Algo {
    FineGrain = 0,
    NonDet,
    Det,
    RUDY,
    PIN_DENSITY,
    DRC_MAP
};

static Algo StrToAlgo(string algo_str) {
    if(algo_str == "FineGrain")
        return FineGrain;
    else if(algo_str == "NonDet") 
        return NonDet;
    else if(algo_str == "Det") 
        return Det;
    else if(algo_str == "RUDY") 
        return RUDY;
    else if(algo_str == "PIN_DENSITY") 
        return PIN_DENSITY;
    else if(algo_str == "DRC_MAP")
        return DRC_MAP;
    else {
        std::cout << "unknown algorithm: " << algo_str << std::endl;
        exit(1);
    }

}

} //namespace sproute

#endif