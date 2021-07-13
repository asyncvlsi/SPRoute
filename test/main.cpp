
#include "sproute.h"

//#include "fastroute.h"
//#include "maxflow.h"
#include <phydb/phydb.h>

int main(int argc, char** argv)
{
    string defFileName, lefFileName, outFileName, algo_str;
    int numThreads = 1;
    for(int i = 1; i < argc; i++)
    {
        string tmp(argv[i]);
        if(tmp == "-lef")
        {   
            lefFileName = string(argv[i+1]);
        }
        else if(tmp == "-def")
        {
            defFileName = string(argv[i+1]);
        }
        else if(tmp == "-t")
        {
            numThreads = atoi(argv[i+1]);
        }
        else if(tmp == "-output")
        {   
            outFileName = string(argv[i+1]);
        }
        else if(tmp == "-algo")
        {   
            algo_str = string(argv[i+1]);
        }
    }
    if(defFileName.size() == 0 || defFileName.size() == 0 || outFileName.size() == 0)
    {
        cout << "usage: ./lefdef_SPRoute -lef [LefFile] -def [DefFile] -t [nthreads] -output [Output file] -algo [Algo]" << endl;
        exit(1);
    }
    cout << "reading lef: " << lefFileName <<endl;
    cout << "reading def: " << defFileName<<endl;

    //preprocessSpacingTable();
    galois::SharedMemSys G;

    phydb::PhyDB db;
    db.ReadLef(lefFileName);
    db.ReadDef(defFileName);
    cout << "reading lef/def done" << endl;

    sproute::SPRoute sproute(&db, 1, "NonDet");
    /*sproute.numThreads = numThreads;
    sproute.LoadPhyDB(&db);
    sproute.LinkTrackToLayer();
    sproute.PreprocessSpacingTable(); 
    
    sproute.PreprocessComponent(); 
    sproute.InitGcell();
    
    sproute.PreprocessSNet();
    sproute.PreprocessDesignRule();
    sproute.PreprocessDesignRuleOBS();
    sproute.AdjustGcellCap();

    sproute.InitGrGen();

    sproute.algo = sproute::StrToAlgo(algo_str);
    cout << "running algorithm: " << algo_str << endl;
    
    sproute.RunGlobalRoute(outFileName, 30, sproute.algo);*/

    sproute.Run();

    //sproute.WriteGuideToFile("integration_correct.guide");

    db.WriteGuide("integration_test.guide");

    return 0;
}

