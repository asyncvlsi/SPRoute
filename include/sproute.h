#ifndef PWROUTE_H
#define PWROUTE_H


#include "defDataBase.h"
#include "lefDataBase.h"
#include "grGen.h"
#include "algo.h"
#include "maze.h"

#include "galois/LargeArray.h"
#include <phydb/phydb.h> 

namespace sproute {

class SPRoute {
public:
    phydb::PhyDB* db_ptr_;

    sproute::defDataBase defDB;
    sproute::lefDataBase lefDB;

    sproute::grGenerator grGen;
    galois::InsertBag<sproute::CapReduction> capReductions;

    int acc_count;
    int numThreads;

    SPRoute() {
        acc_count = 0;
        numThreads = 1;
    }
    SPRoute(phydb::PhyDB* p) {
        this->LoadPhyDB(p);
        acc_count = 0;
        numThreads = 1;
    }

    void LinkTrackToLayer();
    void PreprocessSpacingTable();
    void PreprocessComponent();
    void InitGcell();
    
    void PreprocessSNet();

    void PreprocessDesignRule();
    void PreprocessDesignRuleOBS();
    void AdjustGcellCap();
    void InitGrGen();

    void ComputePinLocation(sproute::Component& component);
    void ComputeOBSLocation(sproute::Component& component);
    void GenerateGcellGrid();
    void UpdateGcellGrid(int xstep, int ystep);
    void GcellInsertOBS(int x, int y, int z, sproute::Rect2D<int> rect, int OBSIdx);

    sproute::Range<int> GetTrackRange(int layerIdx, sproute::Rect2D<float> rect, bool expand);
    sproute::Range<int> GetTrackRange(int layerIdx, sproute::Rect2D<int> rect, bool expand);

    sproute::Range<float> RangeIntersection(sproute::Range<float> r1, sproute::Range<float> r2);
    sproute::Range<int> RangeIntersection(sproute::Range<int> r1, sproute::Range<int> r2);

    void RunFastRoute(string OutFileName, int maxMazeRound = 500, Algo algo = Algo::NonDet);

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

    void WriteRoutedGrid(FILE* fp, std::set<sproute::Point3D<int>>& routedGrid);
    void WriteRoute3D(char routingfile3D[]);
    void WriteOverflow(string overflowFile);

    void mazeRouteMSMD(int iter, int expand, float costHeight, int ripup_threshold,
                   int mazeedge_Threshold, bool Ordering, int cost_type, galois::LargeArray<bool>& done,
                   galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>& thread_local_storage)

};

} //namespace sproute


#endif