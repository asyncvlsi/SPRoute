#include "defDataBase.h"
#include "lefDataBase.h"
#include "global.h"
#include "grGen.h"

#include "fastroute.h"
#include "maxflow.h"
#include <phydb/phydb.h>
using namespace parser;

std::stringstream removeComments(ifstream& infile)
{
    stringstream fileStream("");
    string line;
    while(getline(infile, line))
    {
        //cout << "removing comments" << endl;
        if(line.find("#") != std::string::npos)
        {
            //cout << line << endl;
            //cout << line.find("#") << " " << line.size() << endl;
            size_t pos = line.find("#");
            line.erase(line.begin() + pos, line.end()); 
        }
        line.append("\n");
        fileStream << line;
    }
    return fileStream;
}

/*void readDef(string defFileName )
{
    //string filename("ispd18_test1.input.def");

    ifstream infile(defFileName);
    if(!infile.is_open()) {
        cout << "Unable to open file: " << defFileName << endl;
        exit(1);
    }
    
    std::stringstream defStream;
    defStream = removeComments(infile);
    
    int cnt = 0;
    string token, token1, token2;
    string comment;
    bool endOfDesign = false;
    while(!defStream.eof())
    {
        defStream >> token;
        //   cout << token << endl;
        if(token.at(0) == '#')
        {
            getline(defStream, comment);
            continue;
        }
        defKeyword keyword = token2defKeyword(token);
        
        switch(keyword) {
            case VERSION: 
                defStream >> defDB.version;
                cout << defDB.version << endl;
                semicolonCheck(defStream);
                break;
            case DIVIDERCHAR:
                defStream >> defDB.dividerChar;
                cout << defDB.dividerChar << endl;
                semicolonCheck(defStream);
                break;
            case BUSBITCHARS:
                defStream >> defDB.busBitChars;
                cout << defDB.busBitChars << endl;
                semicolonCheck(defStream);
                break;
            case DESIGN:
                defStream >> defDB.designName;
                cout << defDB.designName << endl;
                semicolonCheck(defStream);
                break;
            case UNITS:
                defStream >> token1 >> token2;
                assert(token1 == "DISTANCE");
                assert(token2 == "MICRONS"); 
                defStream >> defDB.dbuPerMicro;
                cout << defDB.dbuPerMicro << endl;
                semicolonCheck(defStream);
                break;

            case DIEAREA:
                defStream >> token1;
                assert(token1 == "(");
                defStream >> defDB.dieArea.lowerLeft.x >> defDB.dieArea.lowerLeft.y;
                defStream >> token2 >> token1;
                assert(token2 == ")");
                assert(token1 == "(");
                defStream >> defDB.dieArea.upperRight.x >> defDB.dieArea.upperRight.y;
                defStream >> token2;
                assert(token2 == ")");
                semicolonCheck(defStream);

                cout << defDB.dieArea.lowerLeft.x << " " << defDB.dieArea.lowerLeft.y << endl;
                cout << defDB.dieArea.upperRight.x << " " <<  defDB.dieArea.upperRight.y << endl;       
                break;
            case ROW:
                defDB.parseRow(defStream);
                break;
            case TRACKS:
                defDB.parseTracks(defStream);
                break;
            case COMPONENTS:
                defDB.parseComponents(defStream);
                break;
            case PINS:
                defDB.parseIOPins(defStream);
                break;
            case SPECIALNETS:
                defDB.parseSpecialNets(defStream);
                break;
            case NETS:
                defDB.parseNets(defStream);
                break;
            case VIAS:
                defDB.parsedefVias(defStream);
                break;
            case GCELLGRID:
                defDB.parseGcellGrid(defStream);
                semicolonCheck(defStream);
                break;
            case END:
                defStream >> token;
                if(token == "DESIGN")
                {
                    cout << "End of design" << endl;
                    endOfDesign = true;
                    break;
                }
                else {
                    cout << "unknown token : " << token << endl;
                    exit(1);
                }
            default:
                cout << "Error: unsupported keyword " << token << endl;
                exit(1);
        }
        cnt++;
        if(endOfDesign)
            break;
    }
}*/


/*void readLef(string lefFileName)
{
    //string filename("ispd18_test1.input.lef");

    ifstream infile(lefFileName);
    if(!infile.is_open()) {
        cout << "Unable to open file: " << lefFileName << endl;
        exit(1);
    }
   
    std::stringstream lefStream;
    lefStream = removeComments(infile);

    int cnt = 0;
    string token, token1, token2;
    string comment;
    string strOBS;
    bool endOfLibrary = false;
    while(!lefStream.eof())
    {
        lefStream >> token;
        //   cout << token << endl;
        lefKeyword keyword = token2lefKeyword(token);
        
        switch(keyword) {
            case lefVERSION: 
                lefStream >> lefDB.version;
                cout << lefDB.version << endl;
                semicolonCheck(lefStream);
                break;
            case lefDIVIDERCHAR:
                lefStream >> lefDB.dividerChar;
                cout << lefDB.dividerChar << endl;
                semicolonCheck(lefStream);
                break;
            case lefBUSBITCHARS:
                lefStream >> lefDB.busBitChars;
                cout << lefDB.busBitChars << endl;
                semicolonCheck(lefStream);
                break;
            case lefMANUFACTURINGGRID:
                lefStream >> lefDB.manufacturingGrid;
                cout << lefDB.manufacturingGrid << endl;
                semicolonCheck(lefStream);
                break;
            case lefUNITS:
                lefDB.parseUnits(lefStream);
                break;
            case lefPROPERTYDEFINITIONS:
                lefDB.parsePROPERTYDEFINITIONS(lefStream);
                break;

            case lefLAYER:
                lefDB.parseLayer(lefStream);
                break;
            case lefMACRO:
                lefDB.parseMacro(lefStream);
                break;
            case lefVIA:
                lefDB.parselefVia(lefStream);
                break;
            
            case lefSITE:
                lefDB.parseSite(lefStream);
                break;

            case lefCLEARANCEMEASURE:
                lefStream >> lefDB.clearanceMeasure;
                semicolonCheck(lefStream);
                break;

            case lefUSEMINSPACING:
                lefStream >> strOBS >> lefDB.useMinSpacing;
                assert(strOBS == "OBS");
                semicolonCheck(lefStream);
                break;
            case lefVIARULE:
                lefDB.parseViaRule(lefStream);
                break;
                
            case lefEND:
                lefStream >> token;
                if(token == "LIBRARY")
                {
                    cout << "End of library" << endl;
                    endOfLibrary = true;
                    break;
                }
                else {
                    cout << "unknown token : " << token << endl;
                    exit(1);
                }
            default:
                cout << "Error: unsupported keyword " << token << endl;
                exit(1);
        }
        cnt++;
        if(endOfLibrary)
            break;
    }

}*/

void readDef(string defFileName) {
    FILE* f;
    int res;

    defrInit();
    defrReset();

    defrInitSession(1);

    defrSetUserData((defiUserData)&defDB);

    defrSetDesignCbk(getDefString);
    defrSetDesignEndCbk(getDefVoid);
    defrSetDieAreaCbk(getDefDieArea);
    defrSetUnitsCbk(getDefUnits);
    defrSetTrackCbk(getDefTracks);
    defrSetComponentCbk(getDefComponents);
    defrSetPinCbk(getDefIOPins);
    
    defrSetSNetCbk(getDefSNets);
    defrSetAddPathToNet();
    defrSetNetCbk(getDefNets);
    
    defrSetViaCbk(getDefVias);
    defrSetGcellGridCbk(getDefGcell);


    if ((f = fopen(defFileName.c_str(),"r")) == 0) {
        cout <<"Couldn't open def file" <<endl;
        exit(2);
    }

    res = defrRead(f, defFileName.c_str(), (defiUserData)&defDB, 1);
    if (res != 0) {
        cout <<"DEF parser returns an error!" <<endl;
        exit(2);
    }
    fclose(f);

    //numPins = readPinCnt;

    defrClear();
}

void readLef(string lefFileName)
{
    FILE* f;
    int res;

    lefrInitSession(1);

    lefrSetUserData ((lefiUserData)&lefDB);

    lefrSetMacroCbk(getLefMacros);
    lefrSetMacroBeginCbk(getLefString);
    lefrSetMacroEndCbk(getLefString);
    lefrSetUnitsCbk(getLefUnits);
    lefrSetManufacturingCbk(getLefManufacturingGrid);
    lefrSetPinCbk(getLefPins);
    lefrSetObstructionCbk(getLefObs);
    lefrSetLayerCbk(getLefLayers);
    lefrSetViaCbk(getLefVias);
    lefrSetViaRuleCbk(getLefViaGenerateRules);

    if ((f = fopen(lefFileName.c_str(),"r")) == 0) {
        cout <<"Couldn't open lef file" <<endl;
        exit(2);
    }

    res = lefrRead(f, lefFileName.c_str(), (lefiUserData)&lefDB);
    if (res != 0) {
        cout <<"LEF parser returns an error!" <<endl;
        exit(2);
    }
    fclose(f);

    lefrClear();
}

int main(int argc, char** argv)
{
    string defFileName, lefFileName, outFileName, algo_str;
    numThreads = 1;
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


    if(!lefFileName.empty())
        readLef(lefFileName);
    if(!defFileName.empty())
        readDef(defFileName);

    //preprocessSpacingTable();
    galois::SharedMemSys G;
    galois::preAlloc(numThreads * 2);

    numThreads = galois::setActiveThreads(numThreads);

    phydb::PhyDB db;
    db.ReadLef(lefFileName);
    db.ReadDef(defFileName);
    cout << "reading lef/def done" << endl;

    SPRoute sproute;
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

    Algo algo;

    algo = StrToAlgo(algo_str);
    cout << "running algorithm: " << algo_str << endl;
    acc_count = 0;
    
    sproute.RunFastRoute(outFileName, 30, algo);


    return 0;
}
