#include "sproute.h"


namespace sproute {

void SPRoute::Run() {
    LinkTrackToLayer();
    PreprocessSpacingTable(); 
    
    PreprocessComponent(); 
    InitGcell();
    
    PreprocessSNet();
    PreprocessDesignRule();
    PreprocessDesignRuleOBS();
    AdjustGcellCap();

    InitGrGen();
    RunGlobalRoute("", max_iteration, algo);
    if(verbose_ > none)
        std::cout << "routing guides written into phydb" << std::endl;

}

}