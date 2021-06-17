#include "defDataBase.h"

extern sproute::lefDataBase lefDB;
extern sproute::defDataBase defDB;

namespace sproute
{

defKeyword token2defKeyword(string token)
{
    if(token == "VERSION")
        return VERSION;
    else if(token == "DIVIDERCHAR")
        return DIVIDERCHAR;
    else if(token == "BUSBITCHARS")
        return BUSBITCHARS;
    else if(token == "DESIGN")
        return DESIGN;
    else if(token == "UNITS")
        return UNITS;
    else if(token == "DIEAREA")
        return DIEAREA;
    else if(token == "ROW")
        return ROW;
    else if(token == "TRACKS")
        return TRACKS;
    else if(token == "COMPONENTS")
        return COMPONENTS;
    else if(token == "PINS")
        return PINS;
    else if(token == "SPECIALNETS")
        return SPECIALNETS;
    else if(token == "VIAS")
        return VIAS;
    else if(token == "NETS")
        return NETS;
    else if(token == "END")
        return END;
    else if(token == "GCELLGRID")
        return GCELLGRID;
    else {
        cout << "Unknown keyword : " << token << endl;
        exit(1);
    }
}

void semicolonCheck(stringstream& infile)
{
    string token;
    infile >> token;
    if(token != ";") {
        string next_token;
        infile >> next_token;
        cout << "Require a ; before " << next_token << endl;
        exit(1);
    }
}

void defDataBase::sprouteow(stringstream& infile)
{
    Row tmpRow;
    string strDO, strBY, strSTEP;
    infile >> tmpRow.name >> tmpRow.siteName;
    infile >> tmpRow.origX >> tmpRow.origY >> tmpRow.siteOrient;
    infile >> strDO >> tmpRow.numX >> strBY >> tmpRow.numY;
    infile >> strSTEP >> tmpRow.stepX >> tmpRow.stepY;
    semicolonCheck(infile);
    assert(strDO == "DO");
    assert(strBY == "BY");
    assert(strSTEP == "STEP");

    /*cout << tmpRow.name << tmpRow.siteName << endl;
    cout << tmpRow.origX << tmpRow.origY << tmpRow.siteOrient << endl;
    cout << tmpRow.numX << tmpRow.numY << endl;
    cout << tmpRow.stepX << tmpRow.stepY << endl;*/

    this->rows.push_back(tmpRow);

}

void defDataBase::parseTracks(stringstream& infile)
{
    Track tmpTrack;
    string strDO, strLayer, strSTEP, token;
    infile >> tmpTrack.direction >> tmpTrack.start;
    infile >> strDO >> tmpTrack.numTracks >> strSTEP >> tmpTrack.step;
    infile >> strLayer;
    assert(strLayer == "LAYER");
    while(1)
    {
        infile >> token;
        if(token == ";")
            break;
        else
            tmpTrack.layerNames.push_back(token);
    }

    assert(strDO == "DO");
    assert(strSTEP == "STEP");

    /*cout << tmpRow.name << tmpRow.siteName << endl;
    cout << tmpRow.origX << tmpRow.origY << tmpRow.siteOrient << endl;
    cout << tmpRow.numX << tmpRow.numY << endl;
    cout << tmpRow.stepX << tmpRow.stepY << endl;*/

    this->tracks.push_back(tmpTrack);

}

void defDataBase::parseGcellGrid(stringstream& infile)
{
    GcellGrid tmpGcellGrid;
    string strDO, strSTEP, token;
    infile >> tmpGcellGrid.direction >> tmpGcellGrid.start;
    infile >> strDO >> tmpGcellGrid.numBoundaries >> strSTEP >> tmpGcellGrid.step;
    assert(strDO == "DO");
    assert(strSTEP == "STEP");

    /*cout << tmpRow.name << tmpRow.siteName << endl;
    cout << tmpRow.origX << tmpRow.origY << tmpRow.siteOrient << endl;
    cout << tmpRow.numX << tmpRow.numY << endl;
    cout << tmpRow.stepX << tmpRow.stepY << endl;*/

    this->gcellGrids.push_back(tmpGcellGrid);

}

Rect2DLayer<int> defDataBase::sprouteect2DLayer(stringstream& infile)
{
    Rect2DLayer<int> tmpRect2DLayer;
    string strLEFT, strRIGHT;
    infile >> tmpRect2DLayer.layer;
    infile >> strLEFT >> tmpRect2DLayer.lowerLeft.x >> tmpRect2DLayer.lowerLeft.y >> strRIGHT;
    assert(strLEFT == "(");
    assert(strRIGHT == ")");

    infile >> strLEFT >> tmpRect2DLayer.upperRight.x >> tmpRect2DLayer.upperRight.y >> strRIGHT;
    assert(strLEFT == "(");
    assert(strRIGHT == ")");
    return tmpRect2DLayer;
}

void defDataBase::parsedefVias(stringstream& infile)
{
    defVia tmpVia;
    infile >> this->numVias;
    semicolonCheck(infile);

    string strDASH, strPLUS, strLEFT, strRIGHT, token;

    for(int i = 0; i < this->numVias; i++)
    {
        infile >> strDASH >> tmpVia.name;
        //cout << "component name: " << tmpComponent.name << endl;
        assert(strDASH == "-");
        
        while(1)
        {
            infile >> strPLUS;
            if(strPLUS == ";")
                break;
            assert(strPLUS == "+");
            infile >> token;
            if(token == "VIARULE")
            {
                infile >> tmpVia.viaRuleName;
            }
            else if(token == "CUTSIZE")
            {
                infile >> tmpVia.cutSize.x >> tmpVia.cutSize.y;
            }
            else if(token == "LAYERS")
            {
                infile >> tmpVia.layers[0] >> tmpVia.layers[1] >> tmpVia.layers[2];
            }
            else if(token == "CUTSPACING")
            {
                infile >> tmpVia.cutSpacing.x >> tmpVia.cutSpacing.y;
            }
            else if(token == "ENCLOSURE")
            {
                infile >> tmpVia.botEnc.x >> tmpVia.botEnc.y >> tmpVia.topEnc.x >> tmpVia.topEnc.y;
            }
            else if(token == "ROWCOL")
            {
                infile >> tmpVia.numCutRows >> tmpVia.numCutCols;
            }
            else if(token == "ORIGIN")
            {
                infile >> tmpVia.origin.x >> tmpVia.origin.y;
            }
            else if(token == "OFFSET")
            {
                infile >> tmpVia.botOffset.x >> tmpVia.botOffset.y >> tmpVia.topOffset.x >> tmpVia.topOffset.y;
            }
            else if(token == "RECT")
            {
                tmpVia.rect2DLayers.push_back(sprouteect2DLayer(infile));
            }
            else if(token == "PATTERN")
            {
                infile >> tmpVia.pattern;
            }
            else {
                cout << "unknown keyword in parsedefVia: " << token << endl;
                exit(1);
            }
        
        }
        tmpVia.idx = this->vias.size();
        this->defVia2idx.insert(std::pair<string, int> (tmpVia.name, tmpVia.idx));
        this->vias.push_back(tmpVia);
        tmpVia.clear();
    }

    string strEND, strVIAS;
    infile >> strEND >> strVIAS;

    assert(strEND == "END");
    assert(strVIAS == "VIAS");
}


void defDataBase::parseComponents(stringstream& infile)
{
    Component tmpComponent;
    infile >> this->numComponents;
    semicolonCheck(infile);

    string strDASH, strPLUS, strLEFT, strRIGHT, token;
    for(int i = 0; i < this->numComponents; i++)
    {
        infile >> strDASH >> tmpComponent.name >> tmpComponent.macroName;
        //cout << "component name: " << tmpComponent.name << endl;
        assert(strDASH == "-");
        
        while(1)
        {
            infile >> strPLUS;
            if(strPLUS == ";")
                break;
            assert(strPLUS == "+");
            infile >> token;
            if(token == "SOURCE")
            {
                infile >> tmpComponent.source;
            }
            else if(token == "PLACED" || token == "FIXED")
            {
                tmpComponent.locationType = token;
                infile >> strLEFT >> tmpComponent.location.x >> tmpComponent.location.y >> strRIGHT >> tmpComponent.orient;
                assert(strLEFT == "(");
                assert(strRIGHT == ")");
            }
            else if(token == "WEIGHT")
                infile >> tmpComponent.weight;
        }
        tmpComponent.idx = this->components.size();
        this->component2idx.insert(std::pair<string, int> (tmpComponent.name, tmpComponent.idx));
        this->components.push_back(tmpComponent);
        tmpComponent.clear();
    }

    for(int i = 0; i < this->numComponents; i++)
    {
        tmpComponent = components.at(i);
        //cout << tmpComponent.name << tmpComponent.macroName ;
        //cout << tmpComponent.source << tmpComponent.locationType << tmpComponent.location.x << tmpComponent.location.y << tmpComponent.orient << endl;
    }
    string strEND, strCOMPONENTS;
    infile >> strEND >> strCOMPONENTS;

    assert(strEND == "END");
    assert(strCOMPONENTS == "COMPONENTS");


}

void defDataBase::parseNets(stringstream& infile)
{
    Net tmpNet;
    infile >> this->numNets;
    semicolonCheck(infile);

    string strDASH, strLEFT, strRIGHT, token, componentName, pinName, strPLUS;
    for(int i = 0; i < this->numNets; i++)
    {
        infile >> strDASH >> tmpNet.name;
        assert(strDASH == "-");

        while(1)
        {
            infile >> strLEFT;
            if(strLEFT != "(")
                break;
            infile >> componentName >> pinName >> strRIGHT;
            tmpNet.componentNames.push_back(componentName);
            tmpNet.pinNames.push_back(pinName);
            assert(strRIGHT == ")");
        }

        strPLUS = strLEFT;
        while(1)
        {   
            if(strPLUS == ";")
                break;
            assert(strPLUS == "+");
            infile >> token;
            if(token == "SOURCE")
            {
                infile >> tmpNet.source;
            }
            else if(token == "USE")
            {
                infile >> tmpNet.use;
            }
            infile >> strPLUS;
        }

        this->nets.push_back(tmpNet);
        tmpNet.clear();
    }

    for(int i = 0; i < this->numNets; i++)
    {
        tmpNet = nets.at(i);
        //cout << tmpNet.name;
        int pinCnt = 0;
        for(auto str : tmpNet.componentNames)
        {
            //cout << pinCnt << " ";
            //cout << str << tmpNet.pinNames.at(pinCnt) << ", ";
            pinCnt++;
        }
        //cout << endl;
    }
    string strEND, strNETS;
    infile >> strEND >> strNETS;

    assert(strEND == "END");
    assert(strNETS == "NETS");


}

void defDataBase::parseIOPins(stringstream& infile)
{   
    IOPin tmpIOPin;
    infile >> this->numIOPins;
    semicolonCheck(infile);
    string strEND, strPINS;
    
    string strDASH, strPLUS, strLEFT, strRIGHT, token;
    for(int i = 0; i < this->numIOPins; i++)
    {
        infile >> strDASH >> tmpIOPin.name;

        assert(strDASH == "-");
        //cout << tmpIOPin.name << endl;
        while(1)
        {
            infile >> strPLUS;
            //cout << strPLUS << endl;
            if(strPLUS == ";")
                break;
            assert(strPLUS == "+");
            infile >> token;
            if(token == "NET")
            {
                infile >> tmpIOPin.netName;
            }
            else if(token == "PLACED" || token == "FIXED")
            {
                infile >> strLEFT >> tmpIOPin.location.x >> tmpIOPin.location.y >> strRIGHT >> tmpIOPin.orient;
                assert(strLEFT == "(");
                assert(strRIGHT == ")");
            }
            else if(token == "DIRECTION")
                infile >> tmpIOPin.direction;
            else if(token == "USE")
                infile >> tmpIOPin.use;
            else if(token == "LAYER")
            {
                infile >> tmpIOPin.layerName;
                infile >> strLEFT >> tmpIOPin.rect.lowerLeft.x >> tmpIOPin.rect.lowerLeft.y >> strRIGHT;
                assert(strLEFT == "(");
                assert(strRIGHT == ")");
                infile >> strLEFT >> tmpIOPin.rect.upperRight.x >> tmpIOPin.rect.upperRight.y >> strRIGHT;
                assert(strLEFT == "(");
                assert(strRIGHT == ")");
            }
        }
        tmpIOPin.idx = this->iopins.size();
        this->iopin2idx.insert(std::pair<string, int> (tmpIOPin.name, tmpIOPin.idx));
        this->iopins.push_back(tmpIOPin);
        tmpIOPin.clear();
    }

    infile >> strEND >> strPINS;
    assert(strEND == "END");
    assert(strPINS == "PINS");

}

void defDataBase::parseSpecialNets(stringstream& infile)
{
    infile >> this->numSpecialNets;
    semicolonCheck(infile);
    string strEND, strSPECIALNETS;
    string token;
    for(int i = 0; i < this->numSpecialNets; i++ )
    {
        while(1)
        {
            infile >> token;
            if(token == ";")
                break;
        }
    }

    infile >> strEND >> strSPECIALNETS;

    assert(strEND == "END");
    assert(strSPECIALNETS == "SPECIALNETS");


}

Gcell& defDataBase::getGcell(int x, int y, int z)
{
    int loc = z * (gcellGridDim.x * gcellGridDim.y) + y * gcellGridDim.x + x;
    return gcells.at(loc);
}

}//namespace sproute


int getDefString(defrCallbackType_e type, const char* str, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  if ((type == defrDesignStartCbkType)) {
    ((sproute::defDataBase*) data)->designName = string(str);

    if (enableOutput) {
      cout <<"DESIGN " << string(str) <<" ;" <<endl;
    }
  }
  return 0;
}

int getDefVoid(defrCallbackType_e type, void* variable, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  if ((type == defrDesignEndCbkType)) {

    if (enableOutput) {
      cout <<"END DESIGN" <<endl;
    }
  }
  return 0;
}

int getDefDieArea(defrCallbackType_e type, defiBox* box, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  if ((type != defrDieAreaCbkType)) {
    cout <<"Type is not defrDieAreaCbkType!" <<endl;
    exit(1);
  }
  ((sproute::defDataBase*) data)->dieArea.set(box->xl(), box->yl(), box->xh(), box->yh());

  if (enableOutput) {
    cout << " DIE AREA : " << endl;
    ((sproute::defDataBase*) data)->dieArea.print();
  }
    
  return 0;
}

int getDefUnits(defrCallbackType_e type, double number, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  ((sproute::defDataBase*) data)->dbuPerMicro = number;
  if (enableOutput) {
    cout <<"UNITS DISTANCE MICRONS " <<((sproute::defDataBase*) data)->dbuPerMicro <<" ;" <<endl;
  }
  return 0;
}


int getDefTracks(defrCallbackType_e type, defiTrack* track, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if ((type != defrTrackCbkType)) {
    cout <<"Type is not defrTrackCbkType!" <<endl;
    exit(1);
  }

  sproute::Track tmpTrack;

  tmpTrack.direction = track->macro();
  tmpTrack.start = track->x();
  tmpTrack.numTracks = track->xNum();
  tmpTrack.step = track->xStep();

  for(int i = 0; i < track->numLayers(); i++)
  {
    string layerName = track->layer(i);
    tmpTrack.layerNames.push_back(layerName);
  }

  ((sproute::defDataBase*) data)->tracks.push_back(tmpTrack);
  

  if(enableOutput)
    tmpTrack.print();

  return 0;
}


int getDefComponents(defrCallbackType_e type, defiComponent* comp, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type != defrComponentCbkType)) {
    cout <<"Type is not defrComponentCbkType!" <<endl;
    exit(1);
  }

  sproute::Component tmpComp;
  tmpComp.name = comp->id();
  tmpComp.macroName = comp->name();
  tmpComp.locationType = (comp->isFixed())? "FIXED" : "PLACED";
  tmpComp.location.x = comp->placementX();
  tmpComp.location.y = comp->placementY();
  tmpComp.orient = string(comp->placementOrientStr());

  tmpComp.idx = ((sproute::defDataBase*) data)->components.size();
  ((sproute::defDataBase*) data)->component2idx.insert( pair<string, int> (tmpComp.name, tmpComp.idx));
  ((sproute::defDataBase*) data)->components.push_back(tmpComp);

  if(enableOutput)
    tmpComp.print();
  
  return 0;
}


int getDefIOPins(defrCallbackType_e type, defiPin* pin, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;
  if (type != defrPinCbkType) {
    cout <<"Type is not defrPinCbkType!" <<endl;
    exit(1);
  }

  sproute::IOPin tmpPin;
  tmpPin.name = pin->pinName();
  tmpPin.netName = pin->netName();
  int llx, lly, urx, ury;
  llx = 0;
  lly = 0;
  urx = 0;
  ury = 0;

  if (pin->hasPort()) {
    cout <<"Error: multiple pin ports existing in DEF" <<endl;
    exit(1);
  } 
  else 
  {
    
    for (int i = 0; i < pin->numLayer(); ++i) 
    {
      tmpPin.layerName = pin->layer(i);
      
      tmpPin.location.x = pin->placementX();
      tmpPin.location.y = pin->placementY();
      tmpPin.orient = string(pin->orientStr());

      pin->bounds(i, &llx, &lly, &urx, &ury);
      tmpPin.rect.set(llx, lly, urx, ury);
    }

  }

  tmpPin.idx = ((sproute::defDataBase*) data)->iopins.size();
  ((sproute::defDataBase*) data)->iopin2idx.insert( pair<string, int> (tmpPin.name, tmpPin.idx));
  ((sproute::defDataBase*) data)->iopins.push_back(tmpPin);

  if(enableOutput)
    tmpPin.print();

  return 0;

}


int getDefNets(defrCallbackType_e type, defiNet* net, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;

  if (type != defrNetCbkType ) {
    cout <<"Type is not defrNetCbkType!" <<endl;
    exit(1);
  }
  sproute::Net tmpNet;
  tmpNet.name = net->name();

  if (enableOutput) {
    cout <<"- " <<net->name();
  }
  for (int i = 0; i < net->numConnections(); i++) {
    tmpNet.componentNames.push_back(string(net->instance(i)));
    tmpNet.pinNames.push_back(string(net->pin(i)));
  }

  int netID = ((sproute::defDataBase*) data)->nets.size();
  ((sproute::defDataBase*) data)->nets.push_back(tmpNet);
  ((sproute::defDataBase*) data)->netName2netidx.insert(pair<string, int>(tmpNet.name, netID));


  if(enableOutput)
    tmpNet.print();

  return 0;
}

int getDefSNets(defrCallbackType_e type, defiNet* net, defiUserData data) {
  bool enableOutput = false;
  //bool enableOutput = true;

  if (type != defrSNetCbkType) {
    cout <<"Type is not defr(S)NetCbkType!" <<endl;
    exit(1);
  }
  sproute::SNet tmpSNet;
  tmpSNet.name = net->name();

  
  // read pre-route
  //cout << "Net " << net->name() << " has " << net->numWires() << " wires\n";
  //cout << "Net " << net->name() << " has " << net->numPaths() << " paths\n"; // no paths
  //cout << "Net " << net->name() << " has " << net->numVpins() << " vpins\n"; // no vpins
  
  // initialize
  string layerName   = "";
  string viaName     = "";
  string shape       = "";
  bool hasBeginPoint = false;
  bool hasEndPoint   = false;
  int beginX     = -1;
  int beginY     = -1;
  int beginExt   = -1;
  int endX       = -1;
  int endY       = -1;
  int endExt     = -1;
  bool hasRect   = false;
  int llx        = -1;
  int lly        = -1;
  int urx        = -1;
  int ury        = -1;
  int width      = 0;
  for (int i = 0; i < (int)net->numWires(); i++) {
    defiWire* tmpWire = net->wire(i);
    //cout << "Wire " << i << "\n";
    //cout << "  Type: " << tmpWire->wireType() << endl;
    //cout << "  has " << tmpWire->numPaths() << " paths\n";
    
    if (enableOutput) {
       cout << "SNET " << tmpSNet.name << endl;
       cout <<"  + " <<tmpWire->wireType();
       cout << "  has " << net->numWires() << " wires\n";
       cout << "  has " << tmpWire->numPaths() << " paths\n";
    }
    // each path is a def line
    for (int j = 0; j < (int)tmpWire->numPaths(); j++) {
      sproute::Path tmpPath;
      defiPath* path     = tmpWire->path(j);
      path->initTraverse();
      // initialize
      layerName     = "";
      viaName       = "";
      shape         = "";
      hasBeginPoint = false;
      hasEndPoint   = false;
      beginX        = -1;
      beginY        = -1;
      beginExt      = -1;
      endX          = -1;
      endY          = -1;
      endExt        = -1;
      hasRect       = false;
      llx           = -1;
      lly           = -1;
      urx           = -1;
      ury           = -1;
      width         = 0;
      //cout <<"path here" <<endl;
      
      int pathId;
      while ((pathId = path->next()) != DEFIPATH_DONE) {
        //cout << "  pathId = " << pathId << endl;
        switch(pathId) {
          case DEFIPATH_LAYER:
            tmpPath.layerName = string(path->getLayer());
            break;
          case DEFIPATH_VIA:
            tmpPath.viaName = string(path->getVia());
            break;
          case DEFIPATH_WIDTH:
            tmpPath.width = path->getWidth();
            break;
          case DEFIPATH_POINT:
            if (!hasBeginPoint) {
              path->getPoint(&beginX, &beginY);
              hasBeginPoint = true;
              tmpPath.begin.x = beginX;
              tmpPath.begin.y = beginY;
            } else {
              path->getPoint(&endX, &endY);
              hasEndPoint = true;
              tmpPath.end.x = endX;
              tmpPath.end.y = endY;
            }
            break;
          case DEFIPATH_FLUSHPOINT:
            if (!hasBeginPoint) {
              path->getFlushPoint(&beginX, &beginY, &beginExt);
              hasBeginPoint = true;
              tmpPath.begin.x = beginX;
              tmpPath.begin.y = beginY;
              tmpPath.beginExt = beginExt;
            } else {
              path->getFlushPoint(&endX, &endY, &endExt);
              hasEndPoint = true;
              tmpPath.end.x = endX;
              tmpPath.end.y = endY;
              tmpPath.endExt = endExt;
            }
            break;
          case DEFIPATH_SHAPE:
            tmpPath.shape = path->getShape();
            break;
          case DEFIPATH_RECT:
            path->getViaRect(&llx, &lly, &urx, &ury);
            tmpPath.rect.set(llx, lly, urx, ury);
            hasRect = true;
            break;
          case DEFIPATH_VIRTUALPOINT:
            if (!hasBeginPoint) {
              path->getVirtualPoint(&beginX, &beginY);
              hasBeginPoint = true;

              tmpPath.begin.x = beginX;
              tmpPath.begin.y = beginY;
            } else {
              path->getVirtualPoint(&endX, &endY);
              hasEndPoint = true;

              tmpPath.end.x = endX;
              tmpPath.end.y = endY;
            }
            break;
          default : cout <<" net " <<net->name() <<" unknown pathId " <<pathId <<endl; break;
        }
      }


      // add via
      /*if (viaName != "") {
        if (((io::sproute*)data)->tech->name2via.find(viaName) == ((io::sproute*)data)->tech->name2via.end()) {
          if (VERBOSE > -1) {
            cout <<"Error: unsupported via: " <<viaName <<endl;
          }
        } else {
          frPoint p;
          if (hasEndPoint) {
            p.set(endX, endY);
          } else {
            p.set(beginX, beginY);
          }
          auto viaDef = ((io::sproute*)data)->tech->name2via[viaName];
          auto tmpP = make_unique<frVia>(viaDef);
          tmpP->setOrigin(p);
          tmpP->addToNet(netIn);
          netIn->addVia(tmpP);
        }
      }*/
    tmpSNet.paths.push_back(tmpPath);
    } // end path
  } // end wire
 
  ((sproute::defDataBase*) data)->snets.push_back(tmpSNet);

  if(enableOutput)
    tmpSNet.print();

  return 0;
}


int getDefVias(defrCallbackType_e type, defiVia* via, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if ((type != defrViaCbkType)) {
    cout <<"Type is not defrViaCbkType!" <<endl;
    exit(1);
  }

  sproute::defVia tmpVia;
  tmpVia.name = via->name();

  // viaRule defined via
  if (via->hasViaRule()) {
    char* viaRuleName;
    char* botLayer;
    char* cutLayer;
    char* topLayer;
    int xSize, ySize, xCutSpacing, yCutSpacing, xBotEnc, yBotEnc, xTopEnc, yTopEnc;

    via->viaRule(&viaRuleName, &xSize, &ySize, &botLayer, &cutLayer, &topLayer,
                 &xCutSpacing, &yCutSpacing, &xBotEnc, &yBotEnc, &xTopEnc, &yTopEnc);
    tmpVia.viaRuleName = viaRuleName;
    tmpVia.cutSize.set(xSize, ySize);
    tmpVia.layers[0] = string(botLayer);
    tmpVia.layers[1] = string(cutLayer);
    tmpVia.layers[2] = string(topLayer);

    tmpVia.cutSpacing.set(xCutSpacing, yCutSpacing);

    tmpVia.botEnc.set(xBotEnc, yBotEnc);
    tmpVia.topEnc.set(xTopEnc, yTopEnc);

    int xOrigin = 0;
    int yOrigin = 0;
    if (via->hasOrigin()) {
      via->origin(&xOrigin, &yOrigin);
    }
    tmpVia.origin.set(xOrigin, yOrigin);

    int xBotOffset = 0;
    int yBotOffset = 0;
    int xTopOffset = 0;
    int yTopOffset = 0;
    if (via->hasOffset()) {
      via->offset(&xBotOffset, &yBotOffset, &xTopOffset, &yTopOffset);
    }
    tmpVia.botOffset.set(xBotOffset, yBotOffset);
    tmpVia.topOffset.set(xTopOffset, yTopOffset);

    int numCutRows = 1;
    int numCutCols = 1;
    if (via->hasRowCol()) {
      via->rowCol(&numCutRows, &numCutCols);
    }
    tmpVia.numCutRows = numCutRows;
    tmpVia.numCutCols = numCutCols;


  } 
  else // RECT defined via
  {
    if (via->numPolygons()) {
      cout <<"Error: unsupport polygon in def via" <<endl;
      exit(1);
    }
    char* layerName;
    int xl;
    int yl;
    int xh;
    int yh;

    for (int i = 0; i < via->numLayers(); ++i) {
      via->layer(i, &layerName, &xl, &yl, &xh, &yh);
      sproute::Rect2DLayer<int> tmpRect2DLayer;
      tmpRect2DLayer.set(layerName, xl, yl, xh, yh);
    }
  }

  tmpVia.idx = ((sproute::defDataBase*) data)->vias.size();
  ((sproute::defDataBase*) data)->defVia2idx.insert( pair<string, int> (tmpVia.name, tmpVia.idx));
  ((sproute::defDataBase*) data)->vias.push_back(tmpVia);

  if(enableOutput)
    tmpVia.print();

  return 0;
}


int getDefGcell(defrCallbackType_e type, defiGcellGrid* gcellGrid, defiUserData data)
{
    bool enableOutput = false;

    sproute::GcellGrid tmpGcellGrid;
    tmpGcellGrid.direction = string(gcellGrid->macro());
    tmpGcellGrid.start = gcellGrid->x();
    tmpGcellGrid.numBoundaries = gcellGrid->xNum();
    tmpGcellGrid.step = gcellGrid->xStep();

    ((sproute::defDataBase*) data)->gcellGrids.push_back(tmpGcellGrid);

    if(enableOutput)
        tmpGcellGrid.print();

    return 0;
}

#ifdef OPENDB
using namespace odb;
void dbDefDieArea(odb::dbBlock* chipBlock)
{
    cout << "chipname : " << chipBlock->getConstName() << endl;
    cout << "defdbu: " << chipBlock->getDbUnitsPerMicron() << endl;
    //
    //
    adsRect rect;
    chipBlock->getDieArea(rect);
    cout << "dieArea: ( " <<  rect.xMin() << " , " << rect.yMin() <<  " ) ( " << rect.xMax() << " , " << rect.yMax() << " ) " << endl;


    defDB.dieArea.lowerLeft.x = rect.xMin();
    defDB.dieArea.lowerLeft.y = rect.yMin();
    defDB.dieArea.upperRight.x = rect.xMax();
    defDB.dieArea.upperRight.y = rect.yMax();
}

void dbDefUnits(dbTech* tech)
{
   defDB.dbuPerMicro = tech->getDbUnitsPerMicron(); 
}

void dbDefTracks(dbBlock* chipBlock)
{
    for(auto trackGrid : chipBlock->getTrackGrids())
    {
        assert(trackGrid->getNumGridPatternsX()); 
        assert(trackGrid->getNumGridPatternsY());
        cout << "numX: " <<  trackGrid->getNumGridPatternsX()  << "numY: " << trackGrid->getNumGridPatternsY() << endl;
        sproute::Track tmpTrackX;
        sproute::Track tmpTrackY;
        tmpTrackX.direction = "X";
        tmpTrackY.direction = "Y";
        trackGrid->getGridPatternX(0, tmpTrackX.start, tmpTrackX.numTracks, tmpTrackX.step); 
        trackGrid->getGridPatternY(0, tmpTrackY.start, tmpTrackY.numTracks, tmpTrackY.step); 

        cout << tmpTrackX.start << " " << tmpTrackX.numTracks << " " << tmpTrackX.step << endl;
        cout << tmpTrackY.start << " " << tmpTrackY.numTracks << " " << tmpTrackY.step << endl;

        auto techLayer = trackGrid->getTechLayer();
        string layerName = techLayer->getConstName();

        tmpTrackX.layerNames.push_back(layerName);
        tmpTrackY.layerNames.push_back(layerName);

        defDB.tracks.push_back(tmpTrackX);
        defDB.tracks.push_back(tmpTrackY);
    }
}

void dbDefGcellGrids(dbBlock* chipBlock)
{
    dbGCellGrid* gcellGrid = chipBlock->getGCellGrid();
    if(gcellGrid == NULL)
    {
        cout << " NO GCELLGRID SPECIFIED IN DEF, WILL GENERATE IN SPROUTE ITSELF" << endl;
        return;
    }
    cout << "grid num X : " << gcellGrid->getNumGridPatternsX() << " grid num Y: " << gcellGrid->getNumGridPatternsY() << endl;
    
    for(int i = 0; i < gcellGrid->getNumGridPatternsX(); i++)
    {
        sproute::GcellGrid tmpGcellGrid;
        tmpGcellGrid.direction = "X";
        gcellGrid->getGridPatternX(i, tmpGcellGrid.start, tmpGcellGrid.numBoundaries, tmpGcellGrid.step); 

        cout << tmpGcellGrid.start << " " << tmpGcellGrid.numBoundaries << " " << tmpGcellGrid.step << endl;


        defDB.gcellGrids.push_back(tmpGcellGrid);
    }
    
    for(int i = 0; i < gcellGrid->getNumGridPatternsY(); i++)
    {
        sproute::GcellGrid tmpGcellGrid;
        tmpGcellGrid.direction = "Y";
        gcellGrid->getGridPatternY(i, tmpGcellGrid.start, tmpGcellGrid.numBoundaries, tmpGcellGrid.step); 

        cout << tmpGcellGrid.start << " " << tmpGcellGrid.numBoundaries << " " << tmpGcellGrid.step << endl;


        defDB.gcellGrids.push_back(tmpGcellGrid);
    }
}

void dbDefComponents(dbBlock* chipBlock)
{
    cout << " Bterms size: " << chipBlock->getBTerms().size() << endl;
    /*for(auto bTerm : chipBlock->getBTerms()) //IO PIN
    {
       
    }*/
    cout << " Iterms size: " << chipBlock->getITerms().size() << endl;
    for(auto iTerm : chipBlock->getITerms())
    {

    }

    cout << " Insts size: " << chipBlock->getInsts().size() << endl;
    
    for(auto comp : chipBlock->getInsts())
    {
        auto master = comp->getMaster();
        if(string(master->getConstName()) == "MEM1" || string(master->getConstName()) == "MEM2")
        {
            cout << "Comp: " << comp->getConstName() << " master: " << master->getConstName() << endl;
            int orig_x, orig_y; 
            comp->getOrigin(orig_x, orig_y);
            cout << "origin: " << orig_x << " " << orig_y << endl;
            
            cout << "orient: " << comp->getOrient().getValue() << endl;
            int loc_x, loc_y;
            comp->getLocation(loc_x, loc_y);
            cout << "loc: " << loc_x << " " << loc_y << endl;
            auto BBox = comp->getBBox();
            cout << "OBS size: " << comp->getMaster()->getObstructions().size() << endl;
            for(auto obs : comp->getMaster()->getObstructions())
            {
                cout << obs->xMin() << " " << obs->yMin() << " " << obs->xMax() << " " << obs->yMax() << " " << obs->getTechLayer()->getConstName() << endl;
            }
        }
    }
    cout << " Obstruction size: " << chipBlock->getObstructions().size() << endl;
    assert(chipBlock->getObstructions().size() == 0);

    cout << " Blockage size: " << chipBlock->getBlockages().size() << endl;
    assert(chipBlock->getBlockages().size() == 0);
    cout << " Net size: " << chipBlock->getNets().size() << endl;

    

    cout << " Via size: " << chipBlock->getVias().size() << endl;
    assert(chipBlock->getVias().size() == 0);
}

#endif

