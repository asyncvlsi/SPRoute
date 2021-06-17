#ifndef LEFDATABASE_H
#define LEFDATABASE_H

#include "header.h"
#include "sprouteDataType.h"

namespace sproute
{

class ViaRuleGenerateLayer
{
public:
    string layerName; 
    Rect2D<float> rect;
    Size2D<float> spacing;
    Size2D<float> enclosureOverhang;
    
    ViaRuleGenerateLayer( ) { }
    void print()
    {
        cout << " layer: " << layerName << endl;
        if(!rect.empty())
        {
            cout << " RECT:";
            rect.print();
        }
        if(!spacing.empty())
        {
            cout << " SPACING: ";
            spacing.print();
        }
        if(!enclosureOverhang.empty())
        {
            cout << " ENC: ";
            enclosureOverhang.print();
        }
    }

};

class ViaRuleGenerate
{
public: 
    string name;
    bool isDefault;
    ViaRuleGenerateLayer layers[3];
    
    ViaRuleGenerate() { 
        isDefault = true;
    }
    void print()
    {
        cout << "VIARULE: " << name << endl;
        for(int i = 0; i < 3; i++)
        {
            layers[i].print();
        }
    }
};

class PropertyDef
{
public:
    string objectType;
    string propName;
    string propType;

    PropertyDef() { }
    PropertyDef(string objType, string propName, string propType) {
        this->objectType    = objType;
        this->propName      = propName;
        this->propType      = propType;
    }
};

class SpacingTable
{
public:
    int numCols;
    int numRows;
    std::vector<float> parallelRunLength;
    std::vector<float> width;
    std::vector<float> spacing;

    SpacingTable() { }
    SpacingTable(int numCols, int numRows) {
        this->numCols = numCols;
        this->numRows = numRows;
    }

    void setSize(int numCols, int numRows) {
        this->numCols = numCols;
        this->numRows = numRows;
    }
    void reset()
    {
        numCols = 0;
        numRows = 0;
        parallelRunLength.clear();
        width.clear();
        spacing.clear();
    }
    void print()
    {
        cout << "numCols: " << numCols << " numRows: " << numRows << endl;
        cout << " parallelRunLength:" ;
        for(auto p : parallelRunLength)
        {
            cout << p << " ";
        }
        cout << endl;
        cout << " width:" ;
        for(auto w : width)
        {
            cout << w << " ";
        }
        cout << endl;
        cout << " spacing:" ;
        for(auto s : spacing)
        {
            cout << s << " ";
        }
        cout << endl;
    }
    float getSpacing(int col, int row)
    {
        return spacing.at(col + row*numCols);
    }
};

class Spacing //only EOL spacing for now
{
public:
    //metal layer
    float spacing;
    float eolWidth;
    float eolWithin;
    float parEdge;
    float parWithin;

    //cut layer
    int adjacentCuts;
    float cutWithin;
    Spacing ()
    {
        spacing = 0;
        eolWidth = 0;
        eolWithin = 0;
        parEdge = 0;
        parWithin = 0;
        adjacentCuts = 0;
        cutWithin = 0;
    }
    void reset()
    {
        spacing = 0;
        eolWidth = 0;
        eolWithin = 0;
        parEdge = 0;
        parWithin = 0;
        adjacentCuts = 0;
        cutWithin = 0;
    }
    void print()
    {
        cout << " spacing: " << spacing << " eolWidth: " << eolWidth << " eolWithin: " << eolWithin << endl;
        cout << " parEdge: " << parEdge << " parWithin: " << parWithin << endl;
        cout << " adjacentCuts: " << adjacentCuts << " cutWithin: " << cutWithin << endl;
    }
};

class Property
{
public:
    string propName;
    string propString;
};

class CornerSpacing
{
public: 
    float eolWidth;
    std::vector<float> width;
    std::vector<float> spacing;

    void reset()
    {
        eolWidth = 0;
        width.clear();
        spacing.clear();
    }
    void print()
    {
        cout << "cornerSpacing eolWidth: " << eolWidth << endl;
        cout << "width: ";
        for(auto w : width)
        {
            cout << " " << w; 
        }
        cout << endl;
        cout << "spacing: ";
        for(auto s : spacing)
        {
            cout << " " << s;
        }
        cout << endl;
    }

};

class Layer
{
public: 
    string name;
    string type;
    int idx;
    //metal layer
    string direction;
    float pitchx;
    float pitchy;
    float width;
    float area;
    float minWidth;
    float offset;

    SpacingTable spacingTable;
    std::vector<Spacing> spacings;
    Spacing maxEOLSpacing;
    CornerSpacing cornerSpacing;

    //cut layer
    float spacing;
    void reset()
    {
        name = "";
        type = "";
        idx = 0;
        direction = "";
        pitchx = 0;
        pitchy = 0;
        width = 0;
        area = 0;
        minWidth = 0;
        offset = 0;
        spacingTable.reset();
        spacings.clear();
        cornerSpacing.reset();
        spacing = 0;
    }
    void print()
    {
        cout << "------------------------------" << endl;
        cout << "Layer: " << name << " type: " << type << " direction: " << direction << " idx: " << idx << endl;
        cout << "pitch: " << pitchx << " " << pitchy << " width:" << width << " area: " << area << endl;
        cout << "minWidth: " << minWidth << " offset: " << offset << " spacing: " << spacing << endl;
        if(spacingTable.parallelRunLength.size())
            spacingTable.print();
        for(auto spacing : spacings)
        {
            spacing.print();
        }
        if(cornerSpacing.width.size())
            cornerSpacing.print();

    }
};



class LayerRect
{
public: 
    string layerName;
    std::vector<Rect2D<float>> rects;

    void print()
    {
        cout << " layer rect : layer " << layerName << endl;
        for(auto rect : rects)
            rect.print();
    }
    void reset()
    {
        layerName == "";
        rects.clear();
    }
};

class lefVia
{
public:
    string name;
    bool isDefault;
    std::vector<LayerRect> layerRects;
    void reset()
    {
        name = "";
        isDefault = false;
        layerRects.clear();
    }
    void print()
    {
        cout << " Via name: " << name << endl;
        for(auto l : layerRects)
            l.print();
    }
};

class Site
{
public:
    string name;
    string className;
    float width;
    float height;
    void print()
    {
        cout << name << " " << className << " " << width << " " << height << endl;
    }
};

class Pin
{
public:
    string name;
    string direction;
    string output_tristate;
    string use;
    string shape;
    float antennaDiffArea;
    string antennaDiffAreaLayer;


    std::vector<string> netExpr;
    std::vector<LayerRect> layerRects; // one layer is an element in the vector
    void print()
    {
        cout << "pin name: " << name << " direction: " << direction << endl;
        for(auto layerRect : layerRects)
            layerRect.print();
    }
};

class Obstacle
{
public: 
    std::vector<LayerRect> layerRects; // one layer is an element in the vector

    void reset()
    {
        layerRects.clear();
    }
    void print()
    {
        for(auto layerRect : layerRects)
            layerRect.print();
    }
};

class Macro
{
public:
    string name;

    Point2D<float> origin;
    Point2D<float> size;

    std::vector<Pin> pins;
    Obstacle obs;
    Macro() {}

    void reset()
    {
        name = "";
        origin.reset();
        size.reset();
        pins.clear();
        obs.reset();
    }

    void setOrigin(float x, float y)
    {
        origin.x = x;
        origin.y = y;
    }

    void setSize(float x, float y)
    {
        size.x = x;
        size.y = y;
    }

    void print()
    {
        cout << "macro name: " << name << endl;

        cout << "origin point: " << endl;
        origin.print();
        cout << "size: " << endl;
        size.print();
        cout << "width: " << size.x << "height: " << size.y << endl;

        cout << "Pins: ";
        for(auto pin : pins)
            pin.print();
    }
};

class Unit 
{
public:
    string types;
    string unit;
    int factors;
    Unit() {}
    Unit(string types, string unit, int factors):
        types(types), unit(unit), factors(factors) {}
};

class lefDataBase 
{
public:
    std::vector<Unit> units;
    std::vector<PropertyDef> properties;
    std::vector<Layer> layers;
    std::vector<lefVia> vias;
    std::vector<Site> sites;
    std::vector<Macro> macros;
    std::vector<ViaRuleGenerate> viaRuleGenerates;

    string version;
    string busBitChars;
    string dividerChar;

    int dbuPerMicron;

    double manufacturingGrid;

    std::map<string, int> layer2idx;
    std::map<string, int> macro2idx;
    std::map<string, int> lefVia2idx;
    std::map<string, int> viaRuleGenerate2idx;

    Macro tmpMacro;
    Layer tmpLayer;

    void parseViaRule(stringstream& );
    void parseUnits(stringstream& );
    void parseLayer(stringstream& );
    void parselefVia(stringstream& );
    void parseMacro(stringstream& );
    void parseSite(stringstream& );
    void parsePROPERTYDEFINITIONS(stringstream& );
    LayerRect parseLayerRect(stringstream& ,string& );
    ViaRuleGenerateLayer parseViaRuleGenerateLayer(stringstream& ,string& );
    SpacingTable parseSpacingTable(stringstream& );
    Pin parsePin(stringstream& );
    Obstacle parseOBS(stringstream& );
};

enum lefKeyword
{
    lefVERSION = 0,
    lefDIVIDERCHAR,
    lefBUSBITCHARS,
    lefMANUFACTURINGGRID,
    lefUNITS,
    lefLAYER,
    lefMACRO,
    lefVIA,
    lefSITE,
    lefEND,
    lefCLEARANCEMEASURE,
    lefUSEMINSPACING,
    lefPROPERTYDEFINITIONS,
    lefVIARULE
};

lefKeyword token2lefKeyword(string );
}

int getLefMacros(lefrCallbackType_e , lefiMacro* , lefiUserData );
int getLefString(lefrCallbackType_e , const char* , lefiUserData );
int getLefUnits(lefrCallbackType_e , lefiUnits* , lefiUserData );
int getLefManufacturingGrid(lefrCallbackType_e , double , lefiUserData );
int getLefPins(lefrCallbackType_e , lefiPin* , lefiUserData );
int getLefObs(lefrCallbackType_e , lefiObstruction* , lefiUserData );
int getLefLayers(lefrCallbackType_e , lefiLayer* , lefiUserData );
int getLefVias(lefrCallbackType_e , lefiVia* , lefiUserData );
int getLefViaGenerateRules(lefrCallbackType_e , lefiViaRule* , lefiUserData );


#endif

