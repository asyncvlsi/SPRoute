#ifndef SPROUTE_H
#define SPROUTE_H


#include "defDataBase.h"
#include "lefDataBase.h"
#include "grGen.h"
#include "algo.h"
#include "maze.h"

#include "galois/LargeArray.h"
#include <phydb/phydb.h> 

namespace sproute {

class SPRoute {
private:
    phydb::PhyDB* db_ptr_;

    sproute_db::defDataBase defDB;
    sproute_db::lefDataBase lefDB;

    sproute_db::grGenerator grGen;
    galois::InsertBag<sproute_db::CapReduction> capReductions;

    Algo algo;

    int acc_count;
    int numThreads;
    int max_iteration;

    void LinkTrackToLayer();
    void PreprocessSpacingTable();
    void PreprocessComponent();
    void InitGcell();
    
    void PreprocessSNet();

    void PreprocessDesignRule();
    void PreprocessDesignRuleOBS();
    void AdjustGcellCap();
    void InitGrGen();

    void ComputePinLocation(sproute_db::Component& component);
    void ComputeOBSLocation(sproute_db::Component& component);
    void GenerateGcellGrid();
    void UpdateGcellGrid(int xstep, int ystep);
    void GcellInsertOBS(int x, int y, int z, sproute_db::Rect2D<int> rect, int OBSIdx);

    sproute_db::Range<int> GetTrackRange(int layerIdx, sproute_db::Rect2D<float> rect, bool expand);
    sproute_db::Range<int> GetTrackRange(int layerIdx, sproute_db::Rect2D<int> rect, bool expand);

    sproute_db::Range<float> RangeIntersection(sproute_db::Range<float> r1, sproute_db::Range<float> r2);
    sproute_db::Range<int> RangeIntersection(sproute_db::Range<int> r1, sproute_db::Range<int> r2);

    void LoadPhyDB(phydb::PhyDB* p);
    void LoadPhyDBToLefDB();
    void LoadPhyDBToDefDB();

    void LoadPhyDBMacros();
    void LoadPhyDBLefVias();
    void LoadPhyDBLayers();

    void LoadPhyDBDieArea();
    void LoadPhyDBComponents();
    void LoadPhyDBTracks();
    void LoadPhyDBIOPins();
    void LoadPhyDBSNets();
    void LoadPhyDBNets();
    void LoadPhyDBGcellGrids();

    void WriteGuideToPhydb();
    void WriteRoutedGridToPhydb(int netID, std::set<sproute_db::Point3D<int>>& routedGrid);

    void WriteGuideToFile(string guideFileName);
    void WriteRoutedGrid(FILE* fp, std::set<sproute_db::Point3D<int>>& routedGrid);
    void WriteOverflow(string overflowFile);
    
    int InvalidNetsAdjLayer(int x, int y, int l, int xGrid, int yGrid);
    void InvalidNetsAdj(Net** invalid_nets, int xGrid, int yGrid);

    void MemPinAdj(int xGrid, int yGrid);

    void ReadGR(sproute_db::grGenerator& grGen, Algo algo);
    void RunGlobalRoute(string OutFileName, int maxMazeRound = 500, Algo algo = Algo::NonDet);

    void mazeRouteMSMD(int iter, int expand, float costHeight, int ripup_threshold,
                   int mazeedge_Threshold, bool Ordering, int cost_type, galois::LargeArray<bool>& done,
                   galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>& thread_local_storage);

    void mazeRouteMSMDDetPart_Astar_Local(int iter, int expand, float costHeight,
                          int ripup_threshold, int mazeedge_Threshold,
                          bool Ordering, int cost_type, int parts,
                          std::vector<std::vector<int>>& vecParts, galois::LargeArray<bool> &done, 
                          galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>& thread_local_storage);

    void mazeRouteMSMD_astar(int iter, int expand, float costHeight, int ripup_threshold,
                   int mazeedge_Threshold, bool Ordering, int cost_type, float astar_weight, galois::LargeArray<bool>& done);

    void PlotRudy(float* rudy, Algo algo);
    float PlotPinDensity(float* pin_density, Algo algo);
    void PlotDrcMap();
    void UndoneFilter(galois::LargeArray<bool> &done);
    void RUDY_scheduler(int iter, int max_overflow, int counted_num_part, std::vector<std::vector<int>>& vecParts, galois::LargeArray<bool> &done);

public:
    ~SPRoute() {}
    SPRoute() {
        acc_count = 0;
        max_iteration = 30;
        numThreads = 1;
        algo = NonDet;
        
        galois::preAlloc(numThreads * 2);
        galois::setActiveThreads(numThreads);
    }
    SPRoute(phydb::PhyDB* p) {
        this->LoadPhyDB(p);
        acc_count = 0;
        max_iteration = 30;
        numThreads = 1;
        algo = NonDet;

        galois::preAlloc(numThreads * 2);
        galois::setActiveThreads(numThreads);
    }
    SPRoute(phydb::PhyDB* p, int numT, std::string algo_str) {
        this->LoadPhyDB(p);
        acc_count = 0;
        max_iteration = 30;
        numThreads = numT;
        algo = sproute::StrToAlgo(algo_str);

        galois::preAlloc(numThreads * 2);
        galois::setActiveThreads(numThreads);
    }


    void SetAlgo(std::string algo_str) {
        algo = sproute::StrToAlgo(algo_str);
    }

    void SetNumThreads(int numT) {
        numThreads = numT;
    }

    void SetMaxIteration(int max_iter) {
        max_iteration = max_iter;
    }

    void Run();


};


} //namespace sproute


#endif