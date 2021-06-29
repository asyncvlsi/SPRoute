#include "defDataBase.h"

namespace sproute_db
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

Gcell& defDataBase::getGcell(int x, int y, int z)
{
    int loc = z * (gcellGridDim.x * gcellGridDim.y) + y * gcellGridDim.x + x;
    return gcells.at(loc);
}




int getDefString(defrCallbackType_e type, const char* str, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  if ((type == defrDesignStartCbkType)) {
    ((defDataBase*) data)->designName = string(str);

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
  ((defDataBase*) data)->dieArea.set(box->xl(), box->yl(), box->xh(), box->yh());

  if (enableOutput) {
    cout << " DIE AREA : " << endl;
    ((defDataBase*) data)->dieArea.print();
  }
    
  return 0;
}

int getDefUnits(defrCallbackType_e type, double number, defiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  ((defDataBase*) data)->dbuPerMicro = number;
  if (enableOutput) {
    cout <<"UNITS DISTANCE MICRONS " <<((defDataBase*) data)->dbuPerMicro <<" ;" <<endl;
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

  Track tmpTrack;

  tmpTrack.direction = track->macro();
  tmpTrack.start = track->x();
  tmpTrack.numTracks = track->xNum();
  tmpTrack.step = track->xStep();

  for(int i = 0; i < track->numLayers(); i++)
  {
    string layerName = track->layer(i);
    tmpTrack.layerNames.push_back(layerName);
  }

  ((defDataBase*) data)->tracks.push_back(tmpTrack);
  

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

  Component tmpComp;
  tmpComp.name = comp->id();
  tmpComp.macroName = comp->name();
  tmpComp.locationType = (comp->isFixed())? "FIXED" : "PLACED";
  tmpComp.location.x = comp->placementX();
  tmpComp.location.y = comp->placementY();
  tmpComp.orient = string(comp->placementOrientStr());

  tmpComp.idx = ((defDataBase*) data)->components.size();
  ((defDataBase*) data)->component2idx.insert( pair<string, int> (tmpComp.name, tmpComp.idx));
  ((defDataBase*) data)->components.push_back(tmpComp);

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

  sproute_db::IOPin tmpPin;
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

  tmpPin.idx = ((sproute_db::defDataBase*) data)->iopins.size();
  ((sproute_db::defDataBase*) data)->iopin2idx.insert( pair<string, int> (tmpPin.name, tmpPin.idx));
  ((sproute_db::defDataBase*) data)->iopins.push_back(tmpPin);

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
  sproute_db::Net tmpNet;
  tmpNet.name = net->name();

  if (enableOutput) {
    cout <<"- " <<net->name();
  }
  for (int i = 0; i < net->numConnections(); i++) {
    tmpNet.componentNames.push_back(string(net->instance(i)));
    tmpNet.pinNames.push_back(string(net->pin(i)));
  }

  int netID = ((sproute_db::defDataBase*) data)->nets.size();
  ((sproute_db::defDataBase*) data)->nets.push_back(tmpNet);
  ((sproute_db::defDataBase*) data)->netName2netidx.insert(pair<string, int>(tmpNet.name, netID));


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
  sproute_db::SNet tmpSNet;
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
      sproute_db::Path tmpPath;
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
 
  ((sproute_db::defDataBase*) data)->snets.push_back(tmpSNet);

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

  sproute_db::defVia tmpVia;
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
      sproute_db::Rect2DLayer<int> tmpRect2DLayer;
      tmpRect2DLayer.set(layerName, xl, yl, xh, yh);
    }
  }

  tmpVia.idx = ((sproute_db::defDataBase*) data)->vias.size();
  ((sproute_db::defDataBase*) data)->defVia2idx.insert( pair<string, int> (tmpVia.name, tmpVia.idx));
  ((sproute_db::defDataBase*) data)->vias.push_back(tmpVia);

  if(enableOutput)
    tmpVia.print();

  return 0;
}

int getDefGcell(defrCallbackType_e type, defiGcellGrid* gcellGrid, defiUserData data)
{
    bool enableOutput = false;

    sproute_db::GcellGrid tmpGcellGrid;
    tmpGcellGrid.direction = string(gcellGrid->macro());
    tmpGcellGrid.start = gcellGrid->x();
    tmpGcellGrid.numBoundaries = gcellGrid->xNum();
    tmpGcellGrid.step = gcellGrid->xStep();

    ((sproute_db::defDataBase*) data)->gcellGrids.push_back(tmpGcellGrid);

    if(enableOutput)
        tmpGcellGrid.print();

    return 0;
}

int find_Gcell(int pin_in, std::vector<int> GcellBoundaries)
{
	auto it = std::upper_bound(GcellBoundaries.begin(), GcellBoundaries.end(), pin_in);
	int x = std::distance(GcellBoundaries.begin(), it) - 1;

	if(x == -1)
		x++;
	else if(it == GcellBoundaries.end())
		x--;

	return x;
}

}//namespace sproute_db



