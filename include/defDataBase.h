#ifndef DEFDATABASE_H
#define DEFDATABASE_H

#include "header.h"
#include "lefDataBase.h"

namespace sproute_db
{

class defVia
{
public:
    string name;
    int idx;
    string viaRuleName;
    Size2D<int> cutSize;
    string layers[3];
    Size2D<int> cutSpacing;
    Size2D<int> botEnc;
    Size2D<int> topEnc;
    int numCutRows;
    int numCutCols;
    Size2D<int> origin;
    Size2D<int> botOffset;
    Size2D<int> topOffset;

    vector<Rect2DLayer<int>> rect2DLayers;
    string pattern;

    defVia( ) { }

    void clear()
    {
        name = "";
        viaRuleName = "";
        cutSize.clear();
        layers[0] = "";
        layers[1] = "";
        layers[2] = "";
        cutSpacing.clear();
        botEnc.clear();
        topEnc.clear();
        numCutRows = 0;
        numCutCols = 0;
        origin.clear();
        botOffset.clear();
        topOffset.clear();
    }
    void print()
    {
        cout << "Via: " << name << " VIARule: " << viaRuleName << endl;
        cout << "CUT: " << cutSize.x << " " << cutSize.y << endl;
        cout << "Layer: " << layers[0] << " " << layers[1] << " " << layers[2] << endl;
        cout << "CUTSPACING: " << cutSpacing.x << " " << cutSpacing.y << endl;
        cout << "botEnc: " << botEnc.x << " " << botEnc.y << endl;
        cout << "topEnc: " << topEnc.x << " " << topEnc.y << endl;
        cout << " row: " << numCutRows << " col: " << numCutCols << endl;
        cout << " origin: " << origin.x << " " << origin.y << endl;
        cout << " botOffset: " << botOffset.x << " " << botOffset.y << endl;
        cout << " topOffset: " << topOffset.x << " " << topOffset.y << endl;
    }
};


class Row
{
public:
    string name;
    string siteName;
    int origX;
    int origY;
    string siteOrient;
    int numX;
    int numY;
    int stepX;
    int stepY;
};

class Track
{
public:
    string direction;
    int start;
    int numTracks;
    int step;
    std::vector<string> layerNames;

    Track()
    {
        direction = "";
        start = 0;
        numTracks = 0;
        step = 0;
    }
    void reset()
    {
        direction = "";
        start = 0;
        numTracks = 0;
        step = 0;
        layerNames.clear();
    }
    void print()
    {
        cout << "TRACKS : " << direction << " start " << start << " DO " << numTracks << " step " << step << " ";
        for(auto layer : layerNames)
            cout << " " << layer;
        cout << endl;
    }
};

class Component
{
public:
    int idx;
    string name;
    string macroName;
    string source;
    string locationType;
    Point2D<int> location;
    string orient;

    int weight;

    Macro macro;
    Component() {}
    void clear()
    {
        idx = -1;
        name = "";
        macroName = "";
        source = "";
        locationType = "";
        location.x = 0;
        location.y = 0;
        orient = "";
    }
    void print() {
        cout << "Comp name:" << name << " Macro name:" << macroName <<endl;
        cout << "location: ";
        location.print();
        cout << "orient: " << orient << endl;
        macro.print(); 
    }
};

class Net
{
public:
    string name;
    string source;
    string use;


    std::vector<string> componentNames;
    std::vector<string> pinNames;
    Net()
    {
        name = "";
        source = "";
        use = "";
    }
    void clear()
    {
        name = "";
        source = "";
        use = "";
        componentNames.clear();
        pinNames.clear();
    }
    void reset()
    {
        name = "";
        source = "";
        use = "";
        componentNames.clear();
        pinNames.clear();
    }
    void print()
    {
        cout << "NET: " << name << endl;
        for(int i = 0; i < componentNames.size(); i++)
            cout << " ( " << componentNames.at(i) << " " << pinNames.at(i) << " ) " ; 
        cout << endl;
    }

};

class Path
{
public:
    string layerName;
    int width;
    string shape;
    string viaName;

    int beginExt;
    int endExt;
    Rect2D<int> rect;
    Point2D<int> begin;
    Point2D<int> end;
    Path()  {
        layerName = "";
        width = 0;
        shape = "";
        viaName = "";
        beginExt = 0;
        endExt = 0;
    }
    void print()
    {
        cout << " NEW " << layerName << " " << width << " + SHAPE " << shape;
        if(!rect.empty())
            cout << " (" << rect.lowerLeft.x << " " << rect.lowerLeft.y << " " << rect.upperRight.x << " " << rect.upperRight.y << ")";
        if(!begin.empty())
            cout << " (" << begin.x << " " << begin.y << " ) ";
        if(!end.empty())
            cout << " (" << end.x << " " << end.y << " ) ";
        if(beginExt != 0 || endExt != 0)
            cout << " EXT " << beginExt << " " << endExt;
        if(viaName != "")
            cout << viaName;
        cout << endl;
    }


};

class SNet
{
public:
    string name;
    
    std::vector<Path> paths;
    void clear()
    {
        name = "";
        paths.clear();
    }
    void print()
    {
        cout << "SNET: " << name << endl;
        for(auto p : paths)
            p.print();
    }
};

class GcellGrid
{
public:
    string direction;
    int start;
    int numBoundaries;
    int step;

    GcellGrid()
    {
        direction = "";
        start = 0;
        numBoundaries = 0;
        step = 0;
    }
    void reset()
    {
        direction = "";
        start = 0;
        numBoundaries = 0;
        step = 0;
    }
    void print()
    {
        cout << " " << direction << " " << start << " DO " << numBoundaries << " STEP " << step << endl; 
    }

};

enum GcellState{
    NO_OBS = 0,
    HAS_OBS,
    FULLYCOVERED
};

class Gcell
{
public: 
    bool trackUsed[15];
    int numPoints;
    int numNodeOffset;
    int numTracks;
    GcellState obs_state;
    std::vector<int> OBSIDlist; // for verification, can be removed

    Gcell()
    {   
        obs_state = NO_OBS;
        numPoints = 0;
        numTracks = 0;
        OBSIDlist.reserve(16);
    }

};

class IOPin
{
public:
    int idx;
    string name;
    string netName;
    string direction;
    string use;

    string layerName;
    Rect2D<int> rect;

    Point2D<int> location;
    string orient;

    void clear()
    {
        idx = -1;
        name = "";
        netName = "";
        direction = "";
        use = "";
        layerName = "";
        rect.set(0,0,0,0);
        location.set(0,0);
        orient = "";
    }
    void print()
    {
        cout << "PIN: " << name << " net: " << netName << " direction: " << direction << endl;
        cout << " layer: " << layerName << " orient: " << orient << endl;
        cout << " RECT: ";
        rect.print();
        cout << " LOCATION: " ;
        location.print();

    }
};

class CapReduction
{
public:
    int x;
    int y;
    int z;
    int newCap;
    CapReduction( )
    {
        x = 0;
        y = 0;
        z = 0;
        newCap = 0;
    }
};


class defDataBase
{
public:
    string version;
    string dividerChar;
    string busBitChars;
    string designName;
    
    Rect2D<int> dieArea;

    Point3D<int> gcellGridDim;
    Point2D<int> commonGcell;

    int dbuPerMicro;
    int numComponents;
    int numIOPins;
    int numSpecialNets;
    int numNets;
    int numVias;

    std::vector<Row> rows;
    std::vector<Track> tracks;
    std::vector<Component> components;
    std::vector<IOPin> iopins;
    std::vector<SNet> snets;
    std::vector<defVia> vias;
    std::vector<Net> nets;
    std::vector<GcellGrid> gcellGrids;
    std::vector<Gcell> gcells;

    std::map<string, int> component2idx;
    std::map<string, int> iopin2idx;
    std::map<string, int> defVia2idx;
    std::map<string, int> layerName2trackidx;
    std::map<string, int> netName2netidx;
    std::map<int , int> layeridx2trackidx;

    std::vector<int> xGcellBoundaries;
    std::vector<int> yGcellBoundaries;

    std::vector<std::vector<Rect2D<float>>> origOBS;
    std::vector<std::vector<Rect2D<int>>> designRuleOBS;
    
    Gcell& getGcell(int x, int y, int z);

};

enum defKeyword {
    VERSION = 0,
    DIVIDERCHAR,
    BUSBITCHARS,
    DESIGN,
    UNITS,
    DIEAREA,
    ROW,
    TRACKS,
    COMPONENTS,
    PINS,
    SPECIALNETS,
    NETS,
    VIAS,
    GCELLGRID,
    END
};

int find_Gcell(int pin_in, std::vector<int> GcellBoundaries);


} //namespace sproute_db

#endif
