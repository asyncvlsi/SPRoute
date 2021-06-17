#include "lefDataBase.h"
#include "defDataBase.h"

extern sproute::lefDataBase lefDB;
extern sproute::defDataBase defDB;

namespace sproute
{

lefKeyword token2lefKeyword(string token)
{
    if(token == "VERSION")
        return lefVERSION;
    else if(token == "DIVIDERCHAR")
        return lefDIVIDERCHAR;
    else if(token == "BUSBITCHARS")
        return lefBUSBITCHARS;
    else if(token == "MANUFACTURINGGRID")
        return lefMANUFACTURINGGRID;
    else if(token == "UNITS")
        return lefUNITS;
    else if(token == "LAYER")
        return lefLAYER;
    else if(token == "MACRO")
        return lefMACRO;
    else if(token == "VIA")
        return lefVIA;
    else if(token == "END")
        return lefEND;
    else if(token == "SITE")
        return lefSITE;
    else if(token == "CLEARANCEMEASURE")
        return lefCLEARANCEMEASURE;
    else if(token == "USEMINSPACING")
        return lefUSEMINSPACING;
    else if(token == "PROPERTYDEFINITIONS")
        return lefPROPERTYDEFINITIONS;
    else if(token == "VIARULE")
        return lefVIARULE;
    else {
        cout << "Unknown keyword: " << token << endl;
        exit(1);
    }
}

void lefDataBase::parseUnits(stringstream& infile)
{
    string type, unit, token;
    int factor;
    Unit tmpUnit;
    while(1)
    {
        infile >> type ;
        if(type == "END")
        {
            infile >> token;
            assert(token == "UNITS");
            break;
        }
        else
        {
            infile >> unit >> factor ;
            semicolonCheck(infile);
        }
        tmpUnit = Unit(type, unit, factor);
        //cout << type << unit << factor << endl;
        units.push_back(tmpUnit);
    }
}

void lefDataBase::parsePROPERTYDEFINITIONS(stringstream& infile)
{
    string type, token;
    while(1)
    {
        infile >> type ;
        if(type == "END")
        {
            infile >> token;
            assert(token == "PROPERTYDEFINITIONS");
            break;
        }
    }
}

void lefDataBase::parseSite(stringstream& infile)
{
    Site tmpSite;
    string strCLASS, strSIZE, strBY, strEND, token;
    infile >> tmpSite.name;

    while(1)
    {
        infile >> token ;
        if(token == "END")
        {
            infile >> token;
            assert(token == tmpSite.name);
            break;
        }
        else if(token == "SIZE")
        {
            infile >> tmpSite.width >> strBY >> tmpSite.height;
            semicolonCheck(infile);
        }
        else if(token == "CLASS")
        {
            infile >> tmpSite.className;
            semicolonCheck(infile);
        }
    }

    assert(strBY == "BY");
    sites.push_back(tmpSite);
}

SpacingTable lefDataBase::parseSpacingTable(stringstream& infile)
{
    string token;
    SpacingTable tmpSpacingTable;
    int numCols = 0;
    int numRows = 0;
    float width, spacing;
    infile >> token;
    if(token == "PARALLELRUNLENGTH")
    {
        while(1) {
            infile >> token;
            //cout << "spacing table debug " << token << endl;
            if(token == "WIDTH")
                break; 
            numCols++;
            tmpSpacingTable.parallelRunLength.push_back(stof(token));
        }

        do {
            infile >> width;
            //cout << "spacing table width debug" << width << endl;
            tmpSpacingTable.width.push_back(width);
            for(int i = 0; i < numCols; i++)
            {
                infile >> spacing;
                //cout << "spacing table spacing debug" << spacing << endl;
                tmpSpacingTable.spacing.push_back(spacing);
            }
            numRows++;
            infile >> token;
        }while(token == "WIDTH");
        assert(token == ";");
    }
    else
    {
        cout << " Unknown token in spacing table : " << token << endl;
        exit(1);
    }
    tmpSpacingTable.numCols = numCols;
    tmpSpacingTable.numRows = numRows;
    /*for(auto number : tmpSpacingTable.parallelRunLength)
        cout << number;
    cout << endl;
    for(auto number : tmpSpacingTable.width)
        cout << number;
    cout << endl;
    for(auto number : tmpSpacingTable.spacing)
        cout << number;
    cout << endl;*/
    return tmpSpacingTable;
}

void lefDataBase::parseLayer(stringstream& infile)
{
    string token, next;
    string layerName;
    infile >> layerName;
    cout << layerName << endl;
    Layer tmpLayer;
    tmpLayer.name = layerName;
    while(1)
    {
        infile >> token;
        cout << token << endl;
        if(token == "END")
        {
            infile >> next;
            assert(next == layerName);
            break;
        }
        else if(token == "DIRECTION")
        {
            infile >> tmpLayer.direction;
            semicolonCheck(infile);
            //cout << tmpLayer.direction << endl;
        }
        else if(token == "TYPE")
        {
            infile >> tmpLayer.type;
            semicolonCheck(infile);
            //cout << tmpLayer.type << endl;
        }
        else if(token == "PITCH")
        {
            string next1, next2;
            
            infile >> next1 >> next2;
            if(next2 == ";")
            {
                tmpLayer.pitchx = stof(next1);
                tmpLayer.pitchy = stof(next1);
            }
            else
            {
                tmpLayer.pitchx = stof(next1);
                tmpLayer.pitchy = stof(next2);
                semicolonCheck(infile);
            }
            //cout << tmpLayer.pitchx << " " << tmpLayer.pitchy << endl;
        }
        else if(token == "WIDTH")
        {
            infile >> tmpLayer.width;
            semicolonCheck(infile);
            //cout << tmpLayer.width << endl;
        }
        else if(token == "OFFSET")
        {
            infile >> tmpLayer.offset;
            semicolonCheck(infile);
            //cout << tmpLayer.width << endl;
        }
        else if(token == "MINWIDTH")
        {
            infile >> tmpLayer.minWidth;
            semicolonCheck(infile);
        }
        else if(token == "AREA")
        {
            infile >> tmpLayer.area;
            semicolonCheck(infile);
            //cout << tmpLayer.area << endl;
        }
        else if(token == "SPACINGTABLE")
        {
            tmpLayer.spacingTable = parseSpacingTable(infile);
        }
        else if(token == "PROPERTY")
        {
            string next;
            while(1)
            {
                infile >> next;
                if(next == ";")
                    break;
            }
        }
        else if(token == "SPACING")
        {
            Spacing tmpSpacing;
            if(tmpLayer.type == "ROUTING") 
            {
                string strENDOFLINE, strWITHIN;
                infile >> tmpSpacing.spacing >> strENDOFLINE >> tmpSpacing.eolWidth;
                infile >> strWITHIN >> tmpSpacing.eolWithin;
                assert(strENDOFLINE == "ENDOFLINE");
                assert(strWITHIN == "WITHIN");
                string next;
                infile >> next;
                if(next != ";")
                {
                    assert(next == "PARALLELEDGE");
                    infile >> tmpSpacing.parEdge >> strWITHIN >> tmpSpacing.parWithin;
                    assert(strWITHIN == "WITHIN");
                    semicolonCheck(infile);
                }
                
                //cout << tmpSpacing.spacing << tmpSpacing.eolWidth << tmpSpacing.eolWithin << endl;
                tmpLayer.spacings.push_back(tmpSpacing);
            }
            else if(tmpLayer.type == "CUT")
            {
                infile >> tmpSpacing.spacing;
                string next;
                while(1)
                {
                    infile >> next;
                    if(next == ";")
                        break;
                }
                //cout << tmpSpacing.spacing << endl;
            }
            else
            {
                cout << "Unsupported layer type and spacing" << endl;
                exit(1);
            }
        }
        else
        {
            cout << "Unknown Layer token: " << token << endl;
            exit(1);
        }
    }
    int layerIdx = this->layers.size();
    this->layer2idx.insert( pair<string, int> (tmpLayer.name, layerIdx));
    this->layers.push_back(tmpLayer);
}

Pin lefDataBase::parsePin(stringstream& infile)
{
    Pin tmpPin;
    infile >> tmpPin.name;
    cout << tmpPin.name << endl;
    string token, next;
    while(1)
    {
        infile >> token;
        if(token == "END")
        {
            infile>> next;
            assert(next == tmpPin.name);
            break;
        }
        else if(token == "DIRECTION")
        {
            infile >> tmpPin.direction;
            string next;
            infile >> next;
            if(next != ";")
            {
                tmpPin.output_tristate = next;
                semicolonCheck(infile);
            }
            //cout << tmpPin.direction << endl;
        }
        else if(token == "SHAPE")
        {
            infile >> tmpPin.shape;
            semicolonCheck(infile);
            //cout << tmpPin.shape << endl;
        }
        else if(token == "NETEXPR")
        {
            while(1)
            {
                infile >> token;
                if(token == ";")
                    break;
                tmpPin.netExpr.push_back(token);
            }
        }
        else if(token == "ANTENNADIFFAREA")
        {
            string strLAYER;
            infile >> tmpPin.antennaDiffArea >> strLAYER >> tmpPin.antennaDiffAreaLayer;
            assert(strLAYER == "LAYER");
            //cout << tmpPin.antennaDiffArea << strLAYER << tmpPin.antennaDiffAreaLayer << endl;
            semicolonCheck(infile);
        }
        else if(token == "USE")
        {
            infile >> tmpPin.use;
            semicolonCheck(infile);
            //cout << tmpPin.use << endl;
        }
        else if(token == "PORT")
        {
            infile >> token;
            while(token != "END")
            {
                assert(token == "LAYER");
                LayerRect tmpLayerRect = parseLayerRect(infile, token);
                tmpPin.layerRects.push_back(tmpLayerRect);
            }
            //if(token == "END")

        }
        else {
            cout << "Unknown token in parsePin: " << token << " at Pin: " << tmpPin.name << endl;
            exit(1);
        }
    }
    return tmpPin;
}

Obstacle lefDataBase::parseOBS(stringstream& infile)
{
    string token;
    Obstacle tmpOBS;
    infile >> token;
    //cout << token << endl;
    while(token != "END")
    {
        assert(token == "LAYER");
        LayerRect tmpLayerRect = parseLayerRect(infile, token);
        tmpOBS.layerRects.push_back(tmpLayerRect);
    }
    return tmpOBS;
}

/*void lefDataBase::parseMacro(stringstream& infile)
{
    Macro tmpMacro;
    infile >> tmpMacro.name;
    cout << "macro name: " << tmpMacro.name << endl;
    string token, next;
    while(1)
    {
        infile >> token;
        cout << token << endl;
        if(token == "END")
        {
            infile >> next;
            assert(next == tmpMacro.name);
            break;
        }
        else if(token == "CLASS")
        {
            string classStr;
            while(1)
            {
                infile >> classStr;
                tmpMacro.classStrVec.push_back(classStr);
                if(classStr == ";")
                    break;
            }
        }
        else if(token == "SITE")
        {
            infile >> tmpMacro.siteName;
            semicolonCheck(infile);
            //cout << tmpMacro.siteName << endl;
        }
        else if(token == "ORIGIN")
        {
            infile >> tmpMacro.originPoint.x >> tmpMacro.originPoint.y;
            semicolonCheck(infile);
            //tmpMacro.originPoint.print();
        }
        else if(token == "FOREIGN")
        {
            infile >> tmpMacro.foreignCellName ;
            infile >> token;
            if(token == ";")
            {

            }
            else
            {
                tmpMacro.foreignPoint.x = stof(token);
                infile >> tmpMacro.foreignPoint.y;
                semicolonCheck(infile);
            }
            //tmpMacro.foreignPoint.print();
        }
        else if(token == "SIZE")
        {
            string strBY;
            infile >> tmpMacro.width >> strBY >> tmpMacro.height;
            semicolonCheck(infile);
            assert(strBY == "BY");
            //cout << tmpMacro.width << tmpMacro.height << endl;
        }
        else if(token == "SYMMETRY")
        {
            while(1) {
                infile >> token;
                if(token == ";")
                    break;
                tmpMacro.symmetry.push_back(token);
            }
        }
        else if(token == "PIN")
        {
            Pin tmpPin = parsePin(infile);
            tmpMacro.pins.push_back(tmpPin);
        }
        else if(token == "OBS")
        {
            tmpMacro.obs = parseOBS(infile);
        }
        else
        {
            cout << "Unknown Layer token: " << token << endl;
            exit(1);
        }
    }
    int macroIdx = this->macros.size();
    macro2idx.insert( pair<string, int> (tmpMacro.name, macroIdx));
    macros.push_back(tmpMacro);

}*/

LayerRect lefDataBase::parseLayerRect(stringstream& infile, string& token)
{
    LayerRect tmpLayerRect;
    infile >> tmpLayerRect.layerName;
    //cout << tmpLayerRect.layerName;
    semicolonCheck(infile);
    while(1)
    {
        infile >> token;
        if(token == "RECT")
        {
            Rect2D<float> tmpRect;
            infile >> tmpRect.lowerLeft.x >> tmpRect.lowerLeft.y;
            infile >> tmpRect.upperRight.x >> tmpRect.upperRight.y;
            
            //tmpRect.print();
            semicolonCheck(infile);
            tmpLayerRect.rects.push_back(tmpRect);
        }
        else
            return tmpLayerRect;
    }
}

/*ViaRuleGenerateLayer lefDataBase::parseViaRuleGenerateLayer(stringstream& infile, string& token)
{
    ViaRuleGenerateLayer tmpViaRuleGenerateLayer;
    infile >> tmpViaRuleGenerateLayer.layerName;
    semicolonCheck(infile);
    while(1)
    {
        infile >> token;
        if(token == "ENCLOSURE")
        {
            infile >> tmpViaRuleGenerateLayer.overhang1 >> tmpViaRuleGenerateLayer.overhang2;
            semicolonCheck(infile);
        }
        else if(token == "WIDTH")
        {
            string strTO;
            infile >> tmpViaRuleGenerateLayer.minWidth >> strTO >> tmpViaRuleGenerateLayer.maxWidth;
            semicolonCheck(infile);
            assert(strTO == "TO");
        }
        else if(token == "RECT")
        {
            infile >> tmpViaRuleGenerateLayer.rect.lowerLeft.x; 
            infile >> tmpViaRuleGenerateLayer.rect.lowerLeft.y; 
            infile >> tmpViaRuleGenerateLayer.rect.upperRight.x; 
            infile >> tmpViaRuleGenerateLayer.rect.upperRight.y; 
            semicolonCheck(infile);
        }
        else if(token == "SPACING")
        {
            string strBY;
            infile >> tmpViaRuleGenerateLayer.spacing.x >> strBY >> tmpViaRuleGenerateLayer.spacing.y;
            semicolonCheck(infile);
        }
        else if(token == "RESISTANCE")
        {
            infile >> tmpViaRuleGenerateLayer.resistance;
            semicolonCheck(infile);
        }
        else if(token == "LAYER" || token == "END")
        {
            break;
        }
        else {
            cout << "unknown keyword in parsing via rule generate layer: " << token << endl;
            exit(1);
        }
    }
    return tmpViaRuleGenerateLayer;
}*/

/*void lefDataBase::parseViaRule(stringstream& infile)
{
    ViaRuleGenerate tmpViaRuleGenerate;
    string token, strGenerate;
    infile >> tmpViaRuleGenerate.name;
    infile >> strGenerate;
    assert(strGenerate == "GENERATE");
    tmpViaRuleGenerate.isDefault = false;
    infile >> token;
    
    int layerCnt = 0;
    while(1)
    {
        if(token == "DEFAULT")
        {
           tmpViaRuleGenerate.isDefault = true;
           infile >> token;
        }
        else if(token == "LAYER")
        {
            tmpViaRuleGenerate.layers[layerCnt] = parseViaRuleGenerateLayer(infile, token);          
            layerCnt++;
        }
        else if(token == "END")
        {
            break;
        }
        else
        {
            cout << "Unknown token in parseVia: " << token << endl;
            exit(1);
        }
    }
    infile >> token;
    assert(token == tmpViaRuleGenerate.name);

    int viaRuleGenerateIdx = this->viaRuleGenerates.size();
    viaRuleGenerate2idx.insert( pair<string, int> (tmpViaRuleGenerate.name, viaRuleGenerateIdx));
    viaRuleGenerates.push_back(tmpViaRuleGenerate);
}*/


void lefDataBase::parselefVia(stringstream& infile)
{
    lefVia tmpVia;
    string token;
    infile >> tmpVia.name;
    tmpVia.isDefault = false;
    infile >> token;
    while(1)
    {
        if(token == "DEFAULT")
        {
           tmpVia.isDefault = true;
           
           infile >> token;
        }
        else if(token == "LAYER")
        {
            LayerRect tmpLayerRect = parseLayerRect(infile, token);
            
            tmpVia.layerRects.push_back(tmpLayerRect);
        }
        else if(token == "END")
        {
            break;
        }
        else
        {
            cout << "Unknown token in parseVia: " << token << endl;
            exit(1);
        }
    }
    infile >> token;
    assert(token == tmpVia.name);

    int viaIdx = this->vias.size();
    lefVia2idx.insert( pair<string, int> (tmpVia.name, viaIdx));
    vias.push_back(tmpVia);
}

}//namespace sproute

int getLefMacros(lefrCallbackType_e type, lefiMacro* macro, lefiUserData data) {
    //bool enableOutput = true;
    bool enableOutput = false;
    if ((type != lefrMacroCbkType)) {
        cout <<"Type is not lefrMacroCbkType!" <<endl;
        exit(2);
    }

    float originX = macro->originX(); 
    float originY = macro->originY();
    float sizeX   = macro->sizeX();
    float sizeY   = macro->sizeY();
    if (enableOutput) {
        cout <<"  ORIGIN " <<originX  <<" " 
                           <<originY  <<" ;" <<endl;
        cout <<"  SIZE   " <<sizeX    <<" " 
                           <<sizeY    <<endl;
    }

    ((sproute::lefDataBase*)data)->tmpMacro.setOrigin(originX, originY);
    ((sproute::lefDataBase*)data)->tmpMacro.setSize(sizeX, sizeY);

    return 0;
}

int getLefString(lefrCallbackType_e type, const char* str, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (type == lefrMacroBeginCbkType) {
    auto &tmpMacro = ((sproute::lefDataBase*) data)->tmpMacro;
    tmpMacro.reset();
    tmpMacro.name = string(str);
    if (enableOutput) {
      cout <<"MACRO " <<tmpMacro.name <<endl;
    }
  } else if (type == lefrMacroEndCbkType) {
    auto &tmpMacro = ((sproute::lefDataBase*)data)->tmpMacro;
    
    if (enableOutput) {
      cout <<"END " <<tmpMacro.name <<" " << ((sproute::lefDataBase*)data)->macros.size() <<endl;
    }
    int macroIdx = ((sproute::lefDataBase*)data)->macros.size();
    ((sproute::lefDataBase*)data)->macro2idx.insert( pair<string, int> (tmpMacro.name, macroIdx));
    ((sproute::lefDataBase*)data)->macros.push_back(tmpMacro);

  } else {
    cout <<"Type is not supported!" <<endl;
    // exit(2);
  }
  return 0;
}


int getLefUnits(lefrCallbackType_e type, lefiUnits* units, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  ((sproute::lefDataBase*) data)->dbuPerMicron = units->databaseNumber();
  if (enableOutput) {
    cout <<"DATABASE MICRONS " << ((sproute::lefDataBase*) data)->dbuPerMicron <<endl;
  }
  return 0;
}


int getLefManufacturingGrid(lefrCallbackType_e type, double number, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  ((sproute::lefDataBase*) data)->manufacturingGrid = number;
  if (enableOutput) {
    cout <<"MANUFACTURINGGRID " <<number <<endl;
  }
  return 0;
}

void checkLargePin(sproute::Pin pin)
{
    float minx = 1000000, miny = 10000000;
    float maxx = -1000000, maxy = -10000000;
    for(auto layerRect : pin.layerRects)
    {
        for(auto rect : layerRect.rects)
        {
            if(rect.lowerLeft.x < minx)
                minx = rect.lowerLeft.x;
            if(rect.lowerLeft.y < miny)
                miny = rect.lowerLeft.y;
            if(rect.upperRight.x > maxx)
                maxx = rect.upperRight.x;
            if(rect.upperRight.y > maxy)
                maxy = rect.upperRight.y;
        }
    }
    if(maxx - minx > 1.5 || maxy - miny > 1.5)
    {
        cout << "large pin: " << pin.name << endl;
        cout << "bbox " << minx << " " << miny << " " << maxx << " " << maxy << endl;
    }
}

int getLefPins(lefrCallbackType_e type, lefiPin* pin, lefiUserData data) {
    bool enableOutput = false;
    //bool enableOutput = true;
    if (type != lefrPinCbkType) {
        cout <<"Type is not lefrPinCbkType!" <<endl;
        exit(1);
    }

    // term
    auto& tmpMacro = ((sproute::lefDataBase*) data)->tmpMacro;

    sproute::Pin tmpPin;
    sproute::LayerRect tmpLayerRect;
    sproute::Rect2D<float> tmpRect;

    tmpPin.name = pin->name();
    tmpPin.use = pin->use();


    if (enableOutput) {
        cout <<"  PIN " <<pin->name() <<endl;
    }


    int numPorts = pin->numPorts();

    for (int i = 0; i < numPorts; ++i) {
        int numItems = pin->port(i)->numItems();
        
        if (enableOutput) {
          cout <<"    PORT" <<endl;
        }

        int isNewLayerRect = true;

        for (int j = 0; j < numItems; ++j) {
            int itemType = pin->port(i)->itemType(j);
            if (itemType == 1) 
            { //layer
                if(isNewLayerRect)
                {
                    isNewLayerRect = false;
                    tmpLayerRect.reset();
                }
                else
                {
                    tmpPin.layerRects.push_back(tmpLayerRect);
                    tmpLayerRect.reset();
                }
                tmpLayerRect.layerName = pin->port(i)->getLayer(j);

                if (enableOutput) {
                    cout <<"    LAYER " << tmpLayerRect.layerName <<" ;" <<endl;
                }
            } 
            else if (itemType == 8) 
            {
                float llx = pin->port(i)->getRect(j)->xl;
                float lly = pin->port(i)->getRect(j)->yl;
                float urx = pin->port(i)->getRect(j)->xh;
                float ury = pin->port(i)->getRect(j)->yh;
                tmpRect.set(llx, lly, urx, ury);

                tmpLayerRect.rects.push_back(tmpRect);

                if (enableOutput) {
                    cout <<"      RECT " <<llx <<" " <<lly <<" "<<urx <<" " <<ury <<" ;" <<endl;
                }      
            } 
            else 
            {
                cout <<"unsupported lefiGeometries!" <<endl;
                continue;
                // exit(2);
            }
        }
        tmpPin.layerRects.push_back(tmpLayerRect);
    }
    tmpMacro.pins.push_back(tmpPin);

    //tmpPin.print();
    if (enableOutput) 
    {
        cout <<"  END " <<pin->name() <<endl;
    }

    //checkLargePin(tmpPin);

    return 0;
}



int getLefObs(lefrCallbackType_e type, lefiObstruction* obs, lefiUserData data) {
    //bool enableOutput = true;
    bool enableOutput = FALSE;

    if (type != lefrObstructionCbkType) {
        cout <<"Type is not lefrObstructionCbkType!" <<endl;
        exit(1);
    }

    auto& tmpMacro = ((sproute::lefDataBase*) data)->tmpMacro;

    sproute::LayerRect tmpLayerRect;
    sproute::Rect2D<float> tmpRect;

    if (enableOutput) {
        cout <<"  OBS" <<endl;
    }

    auto geometry = obs->geometries();
    int numItems  = geometry->numItems();

    int isNewLayerRect = true;
    for (int i = 0; i < numItems; ++i) 
    {
        if (geometry->itemType(i) == lefiGeomLayerE) 
        {
            if(isNewLayerRect)
            {
                isNewLayerRect = false;
                tmpLayerRect.reset();
            }
            else
            {
                tmpMacro.obs.layerRects.push_back(tmpLayerRect);
                tmpLayerRect.reset();
            }
            tmpLayerRect.layerName = geometry->getLayer(i);

            if (enableOutput) {
            cout <<"    LAYER " <<tmpLayerRect.layerName <<" ;" <<endl;
            }
        } 
        else if (geometry->itemType(i) == lefiGeomRectE) 
        {

            float llx = geometry->getRect(i)->xl;
            float lly = geometry->getRect(i)->yl;
            float urx = geometry->getRect(i)->xh;
            float ury = geometry->getRect(i)->yh;
            tmpRect.set(llx, lly, urx, ury);

            tmpLayerRect.rects.push_back(tmpRect);

            if (enableOutput) {
                cout <<"      RECT " <<llx <<" " <<lly <<" "  <<urx <<" " <<ury <<" ;" <<endl;
            }
        } 
        else 
        {
            cout <<"Warning: unsupported OBS" <<endl;
            continue;
        }
    }
    tmpMacro.obs.layerRects.push_back(tmpLayerRect);

    return 0;
}


int getLefCornerSpacing(void* data, const string& stringProp)
{
    istringstream istr(stringProp);
    string token;
    auto& tmpLayer = ((sproute::lefDataBase*) data)->tmpLayer;
    while(!istr.eof())
    {
        istr >> token;
        if(token == "EXCEPTEOL")
        {
            istr >> tmpLayer.cornerSpacing.eolWidth;
        }
        else if(token == "WIDTH")
        {
            float width;
            istr >> width;
            tmpLayer.cornerSpacing.width.push_back(width);
        }
        else if(token == "SPACING")
        {
            float spacing;
            istr >> spacing;
            tmpLayer.cornerSpacing.spacing.push_back(spacing);
        }
    }
    return 0;
}

int getLefLayers(lefrCallbackType_e type, lefiLayer* layer, lefiUserData data) {

  bool enableOutput = false;

  if (type != lefrLayerCbkType) {
    cout <<"Type is not lefrLayerCbkType!" <<endl;
    exit(1);
  }

  auto& tmpLayer = ((sproute::lefDataBase*) data)->tmpLayer;
  tmpLayer.reset();

  if (strcmp(layer->type(), "ROUTING") == 0) 
  {

    tmpLayer.name = layer->name();
    tmpLayer.type = layer->type();
    tmpLayer.width = layer->width();
    if (layer->hasMinwidth()) {
        tmpLayer.minWidth = layer->minwidth();
    } else {
        tmpLayer.minWidth = layer->width();
    }
    tmpLayer.direction = layer->direction();

    if (layer->hasXYPitch()) {
        tmpLayer.pitchx = layer->pitchX();
        tmpLayer.pitchy = layer->pitchY();
    } else {
        tmpLayer.pitchx = layer->pitch();
        tmpLayer.pitchy = layer->pitch();
    }
    tmpLayer.offset = layer->offset();

    // read minArea rule
    if (layer->hasArea()) {
      tmpLayer.area = layer->area();
    }

    if(enableOutput)
        cout <<"Layer" << layer->name() << " number of props " <<layer->numProps() <<endl;
    if(layer->numProps() > 1)
    {
        cout << "Error: enable to handle more than one properties:" << layer->name() << endl;
    }
    for (int i = 0; i < layer->numProps(); i++) {
        if (string(layer->propName(i)) == string("LEF58_CORNERSPACING") && layer->propIsString(i)) 
        {
            getLefCornerSpacing(data, layer->propValue(i));
        } 
        else 
        {
            cout << "UNSUPPORTED PROPERTY: " << layer->propName(i) << endl;
        }
    }

    // read spacing rule
    for (int i = 0; i < layer->numSpacing(); ++i) {
        sproute::Spacing tmpSpacing;
        tmpSpacing.spacing = layer->spacing(i);

        if (layer->hasSpacingEndOfLine(i)) {

            if (enableOutput) {
                cout <<"  SPACING " <<layer->spacing(i) <<" ENDOFLINE " <<layer->spacingEolWidth(i)
                <<" WITHIN " <<layer->spacingEolWithin(i);
            }
            tmpSpacing.eolWidth = layer->spacingEolWidth(i);
            tmpSpacing.eolWithin = layer->spacingEolWithin(i);

            if (layer->hasSpacingParellelEdge(i)) {
                if (enableOutput) {
                    cout <<" PARALLELEDGE " <<layer->spacingParSpace(i) <<" WITHIN " <<layer->spacingParWithin(i);
                }
                tmpSpacing.parEdge = layer->spacingParSpace(i);
                tmpSpacing.parWithin = layer->spacingParWithin(i);
            }

            tmpLayer.spacings.push_back(tmpSpacing);

        } 
        else { 
            cout << "unsupported spacing!"<<endl;
        }
    }

    // read spacingTable
    if(layer->numSpacingTable() > 1)
        cout << "Error: More than one spacing table!" << endl;

    for (int i = 0; i < layer->numSpacingTable(); ++i) {
        auto spTable = layer->spacingTable(i);
        //if (spTable->isParallel()) {
        if (spTable->parallel()) {
            auto parallel = spTable->parallel();

            if (enableOutput) {
                cout <<"  SPACINGTABLE" <<endl;
                cout <<"  PARALLELRUNLENGTH";
            }
            for (int j = 0; j < parallel->numLength(); ++j) {
                tmpLayer.spacingTable.parallelRunLength.push_back(parallel->length(j));
            }
            for (int j = 0; j < parallel->numWidth(); ++j) {
                tmpLayer.spacingTable.width.push_back(parallel->width(j));
                for (int k = 0; k < parallel->numLength(); ++k) {
                    tmpLayer.spacingTable.spacing.push_back( parallel->widthSpacing(j, k));
                }
            }
            if (enableOutput) {
                cout <<" ;" <<endl;
            }
        } 
        else 
        { 
            cout << "unsupported spacing table!" << endl;
        }
    }

    int layerIdx = ((sproute::lefDataBase*)data)->layers.size();
    tmpLayer.idx = layerIdx;
    ((sproute::lefDataBase*)data)->layer2idx.insert( pair<string, int> (tmpLayer.name, layerIdx));
    ((sproute::lefDataBase*)data)->layers.push_back(tmpLayer);

  } 
  else if (strcmp(layer->type(), "CUT") == 0) // cut layer
  { 

    tmpLayer.name  = layer->name();
    tmpLayer.type  = layer->type();
    tmpLayer.width  = layer->width();
    
    // read spacing constraint
    for (int i = 0; i < layer->numSpacing(); ++i) {

        if(layer->hasSpacingAdjacent(i))
        {
            sproute::Spacing tmpSpacing;
            tmpSpacing.spacing = layer->spacing(i);
            tmpSpacing.adjacentCuts = layer->spacingAdjacentCuts(i);
            tmpSpacing.cutWithin = layer->spacingAdjacentWithin(i);
            tmpLayer.spacings.push_back(tmpSpacing);
        }
        else
        {
            tmpLayer.spacing = layer->spacing(i);
        }
    }
    
    int layerIdx = ((sproute::lefDataBase*) data)->layers.size();
    tmpLayer.idx = layerIdx;
    ((sproute::lefDataBase*) data)->layer2idx.insert( pair<string, int> (tmpLayer.name, layerIdx));
    ((sproute::lefDataBase*) data)->layers.push_back(tmpLayer);

  } 
  else 
  {
    if(string(layer->name()) != "OVERLAP")
        cout << "unsupported layer type: " << layer->name() << ": " << layer->type() << endl;

  }

  if(enableOutput)
    tmpLayer.print();

  return 0;
}



int getLefVias(lefrCallbackType_e type, lefiVia* via, lefiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != lefrViaCbkType) {
    cout <<"Type is not lefrViaCbkType!" <<endl;
    // exit(1);
  }
  sproute::lefVia tmpVia;

  tmpVia.name = via->name();

  if (enableOutput) {
    cout <<"VIA " <<via->name();
    if (via->hasDefault()) {
      cout <<" DEFAULT";
    }
    cout <<endl;
  }
  if (via->numLayers() != 3) {
    cout <<"Error: unsupported via: " << via->name() << endl;
    exit(1);
  }

  for (int i = 0; i < via->numLayers(); ++i) {
    sproute::LayerRect tmpLayerRect;
    tmpLayerRect.layerName = via->layerName(i);
    for (int j = 0; j < via->numRects(i); ++j)
    {
        sproute::Rect2D<float> rect;
        rect.set(via->xl(i, j), via->yl(i, j), via->xh(i, j), via->yh(i, j));
        tmpLayerRect.rects.push_back(rect);
    }
    tmpVia.layerRects.push_back(tmpLayerRect);
  }
  
  int viaIdx = ((sproute::lefDataBase*) data)->vias.size();
  ((sproute::lefDataBase*) data)->lefVia2idx.insert( pair<string, int> (tmpVia.name, viaIdx));
  ((sproute::lefDataBase*) data)->vias.push_back(tmpVia);

  if(enableOutput)
    tmpVia.print();  
  return 0;
}


int getLefViaGenerateRules(lefrCallbackType_e type, lefiViaRule* viaRule, lefiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != lefrViaRuleCbkType) {
    cout <<"Type is not lefrViaRuleCbkType!" <<endl;
    // exit(1);
  }
  sproute::ViaRuleGenerate tmpViaRule;

  if (viaRule->numLayers() != 3) 
  {
    cout <<"Error: unsupported via" <<endl;
    exit(1);
  }
  tmpViaRule.name = viaRule->name();

  for (int i = 0; i < viaRule->numLayers(); ++i) 
  {
    auto viaRuleLayer = viaRule->layer(i);
    auto& layer = tmpViaRule.layers[i];

    layer.layerName = viaRuleLayer->name();
    if(viaRuleLayer->hasEnclosure())
    {
        layer.enclosureOverhang.x = viaRuleLayer->enclosureOverhang1();
        layer.enclosureOverhang.y = viaRuleLayer->enclosureOverhang2();
    }

    if(viaRuleLayer->hasRect())
    {
        layer.rect.set(viaRuleLayer->xl(), viaRuleLayer->yl(), viaRuleLayer->xh(), viaRuleLayer->xh());
    }

    if(viaRuleLayer->hasSpacing())
    {
        layer.spacing.x = viaRuleLayer->spacingStepX();
        layer.spacing.y = viaRuleLayer->spacingStepY();
    }
    
  }
  
  int viaRuleIdx = ((sproute::lefDataBase*) data)->viaRuleGenerates.size();
  ((sproute::lefDataBase*) data)->viaRuleGenerate2idx.insert( pair<string, int> (tmpViaRule.name, viaRuleIdx));
  ((sproute::lefDataBase*) data)->viaRuleGenerates.push_back(tmpViaRule);

  if(enableOutput)
    tmpViaRule.print();

  return 0;
}


#ifdef OPENDB
using namespace odb;
void dbLefLayers(odb::dbTech* tech)
{

    for(odb::dbSetIterator<dbTechLayer> dbLayer = tech->getLayers().begin(); dbLayer != tech->getLayers().end(); dbLayer++)
    {
        string layerName(dbLayer->getConstName());
        uint width = dbLayer->getWidth();
        int spacing = dbLayer->getSpacing();
        int pitch = dbLayer->getPitch();
        // cout << "layer: " << layerName << " " << width << " " << spacing << " " << pitch << endl;
        // int owidth = 0, olength = 0;
        // dbLayer->getMaxWideDRCRange(owidth, olength);
        // cout << owidth << " " << olength << endl;
        // dbLayer->getMinWideDRCRange(owidth, olength);
        // cout << owidth << " " << olength << endl;

        string type, direction;
        if(dbLayer->getType().getValue() == odb::dbTechLayerType::Value::ROUTING)
        {
            type = "ROUTING";
            if(dbLayer->getDirection().getValue() == odb::dbTechLayerDir::Value::HORIZONTAL)
                direction = "HORIZONTAL";
            else if(dbLayer->getDirection().getValue() == odb::dbTechLayerDir::Value::VERTICAL)
                direction = "VERTICAL";
            else 
            {
                cout << "unsupported layer type: " << dbLayer->getType().getValue() << endl;
                exit(1);
            }
        }
        else if(dbLayer->getType().getValue() == odb::dbTechLayerType::Value::CUT)
        {
            type = "CUT";
        }
        else if(dbLayer->getType().getValue() == odb::dbTechLayerType::Value::OVERLAP)
        {
            
        }
        else
        {
            cout << "unsupported layer type: " << dbLayer->getType().getValue() << endl;
            exit(1);
        }
        //cout << "type: " << type << " direction: " << direction << endl;

        /*std::vector< std::vector<uint> > tmpSpTable;
        if(dbLayer->hasV55SpacingRules())
        {
            cout << "has rule" << endl; 
            dbLayer->getV55SpacingTable(tmpSpTable);
            for(auto vec : tmpSpTable)
            {
                for(auto spValue : vec)
                {
                    cout << " " << spValue ; 
                }
                cout << endl;
            }
            cout << "little try: " << dbLayer->getSpacing(1600, 1000) << endl;
        }*/

        sproute::Layer tmpLayer;
        tmpLayer.name = layerName;
        tmpLayer.type = type;
        tmpLayer.direction = direction;
        tmpLayer.pitchx = pitch;
        tmpLayer.pitchy = pitch;
        tmpLayer.width = width;

        int layerIdx = lefDB.layers.size();
        tmpLayer.idx = layerIdx;
        lefDB.layer2idx.insert( pair<string, int> (tmpLayer.name, layerIdx));
        lefDB.layers.push_back(tmpLayer);
    }
}

void dbLefMacros(odb::dbLib* lib)
{
    cout << "lib name: " << lib->getConstName() << endl;

    for(auto dbMacro : lib->getMasters())
    {
        sproute::Macro tmpMacro;
        tmpMacro.name = dbMacro->getConstName();
        int tmpX, tmpY;
        dbMacro->getOrigin(tmpX, tmpY);
        tmpMacro.origin.x = tmpX;
        tmpMacro.origin.y = tmpY;
        tmpMacro.size.x = dbMacro->getWidth();
        tmpMacro.size.y = dbMacro->getHeight();
        //cout << dbMacro->getConstName() << endl;
        //cout << dbMacro->getOrigin(x, y) << dbMacro.getWidth() << dbMacro.getHeight() << endl;
        for(auto dbMTerm : dbMacro->getMTerms())
        {
            sproute::Pin tmpPin;
            tmpPin.name = dbMTerm->getConstName();

            //cout << " " << dbMTerm->getConstName() << endl;
            for(auto dbMPin : dbMTerm->getMPins()) //actually one pin per term, don't know why, maybe useful in def.
            {
                for(auto dbBox : dbMPin->getGeometry())
                {
                    sproute::Rect2D<float> rect;
                    rect.lowerLeft.x = dbBox->xMin();
                    rect.lowerLeft.y = dbBox->yMin();
                    //rect.lowerLeft.z = lefDB.layer2idx.find(dbBox->getTechLayer()->getConstName())->second;
                    rect.upperRight.x = dbBox->xMax();
                    rect.upperRight.y = dbBox->yMax();
                    //rect.upperRight.z = lefDB.layer2idx.find(dbBox->getTechLayer()->getConstName())->second;
                    sproute::LayerRect tmpLayerRect;
                    tmpLayerRect.layerName = dbBox->getTechLayer()->getConstName();
                    tmpLayerRect.rects.push_back(rect);
                    tmpPin.layerRects.push_back(tmpLayerRect);
                    //cout << " pin shape: " << dbBox->xMin() << " " << dbBox->yMin() << " " << dbBox->xMax() << " " << dbBox->yMax() << " " << dbBox->getTechLayer()->getConstName() << endl;
                }
            }
            // for(auto dbTarget : dbMTerm->getTargets())
            // {
            //     cout << " target layer: " << dbTarget->getTechLayer()->getConstName() << endl;
            //     cout << " adsPoint: " << dbTarget->getPoint().getX() << " " << dbTarget->getPoint().getX() << endl;
            // }
            tmpMacro.pins.push_back(tmpPin);
        }

        for(auto dbOBS : dbMacro->getObstructions())
        {
            sproute::Rect2D<float> rect;
            rect.lowerLeft.x = dbOBS->xMin();
            rect.lowerLeft.y = dbOBS->yMin();
            //rect.lowerLeft.z = lefDB.layer2idx.find(dbOBS->getTechLayer()->getConstName());
            rect.upperRight.x = dbOBS->xMax();
            rect.upperRight.y = dbOBS->yMax();
            //rect.upperRight.z = lefDB.layer2idx.find(dbOBS->getTechLayer()->getConstName());
            sproute::LayerRect tmpLayerRect;
            tmpLayerRect.layerName = dbOBS->getTechLayer()->getConstName();
            tmpLayerRect.rects.push_back(rect);
            tmpMacro.obs.layerRects.push_back(tmpLayerRect);
            //cout << " OBS: " << dbOBS->xMin() << " " << dbOBS->yMin() << " " << dbOBS->xMax() << " " << dbOBS->yMax() << " " << dbOBS->getTechLayer()->getConstName() << endl;
        }
    
        int macroIdx = lefDB.macros.size();
        //tmpMacro.idx = macroIdx;
        lefDB.macro2idx.insert( pair<string, int> (tmpMacro.name, macroIdx));
        lefDB.macros.push_back(tmpMacro);
    }

}


void dbLefVias(odb::dbTech* tech)
{
    cout << " via: " << endl;

    for(auto dbTechVia : tech->getVias())
    {
        sproute::lefVia tmpVia;
        tmpVia.name = dbTechVia->getConstName();
        for(auto dbBox : dbTechVia->getBoxes())
        {
            sproute::Rect2D<float> rect;
            rect.lowerLeft.x = dbBox->xMin();
            rect.lowerLeft.y = dbBox->yMin();
            //rect.lowerLeft.z = lefDB.layer2idx.find(dbOBS->getTechLayer()->getConstName());
            rect.upperRight.x = dbBox->xMax();
            rect.upperRight.y = dbBox->yMax();
            //rect.upperRight.z = lefDB.layer2idx.find(dbOBS->getTechLayer()->getConstName());
            sproute::LayerRect tmpLayerRect;
            tmpLayerRect.layerName = dbBox->getTechLayer()->getConstName();
            tmpLayerRect.rects.push_back(rect);
            
            
            tmpVia.layerRects.push_back(tmpLayerRect);
            //cout << " pin shape: " << dbBox->xMin() << " " << dbBox->yMin() << " " << dbBox->xMax() << " " << dbBox->yMax() << " " << dbBox->getTechLayer()->getConstName() << endl;
        }

        int viaIdx = lefDB.vias.size();
        //tmpVia.idx = viaIdx;
        lefDB.lefVia2idx.insert( pair<string, int> (tmpVia.name, viaIdx));
        lefDB.vias.push_back(tmpVia); 
    }
    
}

void dbLefViaRules(odb::dbTech* tech)
{
    cout << "viarule: " << endl;

    for(auto dbTechViaRule : tech->getViaRules())
    {
        cout << dbTechViaRule->getName() << endl;
        
    }
}

void dbLefViaGenerateRules(odb::dbTech* tech)
{
    cout << "via rule generate: " << endl;

    for(auto dbTechViaGenerateRule : tech->getViaGenerateRules())
    {
        cout << dbTechViaGenerateRule->getName() << endl;
    }
}
#endif
