#include "lefDataBase.h"
#include "defDataBase.h"

namespace sproute_db
{





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

    ((sproute_db::lefDataBase*)data)->tmpMacro.setOrigin(originX, originY);
    ((sproute_db::lefDataBase*)data)->tmpMacro.setSize(sizeX, sizeY);

    return 0;
}

int getLefString(lefrCallbackType_e type, const char* str, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  if (type == lefrMacroBeginCbkType) {
    auto &tmpMacro = ((sproute_db::lefDataBase*) data)->tmpMacro;
    tmpMacro.reset();
    tmpMacro.name = string(str);
    if (enableOutput) {
      cout <<"MACRO " <<tmpMacro.name <<endl;
    }
  } else if (type == lefrMacroEndCbkType) {
    auto &tmpMacro = ((sproute_db::lefDataBase*)data)->tmpMacro;
    
    if (enableOutput) {
      cout <<"END " <<tmpMacro.name <<" " << ((sproute_db::lefDataBase*)data)->macros.size() <<endl;
    }
    int macroIdx = ((sproute_db::lefDataBase*)data)->macros.size();
    ((sproute_db::lefDataBase*)data)->macro2idx.insert( pair<string, int> (tmpMacro.name, macroIdx));
    ((sproute_db::lefDataBase*)data)->macros.push_back(tmpMacro);

  } else {
    cout <<"Type is not supported!" <<endl;
    // exit(2);
  }
  return 0;
}


int getLefUnits(lefrCallbackType_e type, lefiUnits* units, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = true;
  ((sproute_db::lefDataBase*) data)->dbuPerMicron = units->databaseNumber();
  if (enableOutput) {
    cout <<"DATABASE MICRONS " << ((sproute_db::lefDataBase*) data)->dbuPerMicron <<endl;
  }
  return 0;
}


int getLefManufacturingGrid(lefrCallbackType_e type, double number, lefiUserData data) {
  //bool enableOutput = true;
  bool enableOutput = false;
  ((sproute_db::lefDataBase*) data)->manufacturingGrid = number;
  if (enableOutput) {
    cout <<"MANUFACTURINGGRID " <<number <<endl;
  }
  return 0;
}

void checkLargePin(sproute_db::Pin pin)
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
    auto& tmpMacro = ((sproute_db::lefDataBase*) data)->tmpMacro;

    sproute_db::Pin tmpPin;
    sproute_db::LayerRect tmpLayerRect;
    sproute_db::Rect2D<float> tmpRect;

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

    auto& tmpMacro = ((sproute_db::lefDataBase*) data)->tmpMacro;

    sproute_db::LayerRect tmpLayerRect;
    sproute_db::Rect2D<float> tmpRect;

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
    auto& tmpLayer = ((sproute_db::lefDataBase*) data)->tmpLayer;
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

  auto& tmpLayer = ((sproute_db::lefDataBase*) data)->tmpLayer;
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
        sproute_db::Spacing tmpSpacing;
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

    int layerIdx = ((sproute_db::lefDataBase*)data)->layers.size();
    tmpLayer.idx = layerIdx;
    ((sproute_db::lefDataBase*)data)->layer2idx.insert( pair<string, int> (tmpLayer.name, layerIdx));
    ((sproute_db::lefDataBase*)data)->layers.push_back(tmpLayer);

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
            sproute_db::Spacing tmpSpacing;
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
    
    int layerIdx = ((sproute_db::lefDataBase*) data)->layers.size();
    tmpLayer.idx = layerIdx;
    ((sproute_db::lefDataBase*) data)->layer2idx.insert( pair<string, int> (tmpLayer.name, layerIdx));
    ((sproute_db::lefDataBase*) data)->layers.push_back(tmpLayer);

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
  sproute_db::lefVia tmpVia;

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
    sproute_db::LayerRect tmpLayerRect;
    tmpLayerRect.layerName = via->layerName(i);
    for (int j = 0; j < via->numRects(i); ++j)
    {
        sproute_db::Rect2D<float> rect;
        rect.set(via->xl(i, j), via->yl(i, j), via->xh(i, j), via->yh(i, j));
        tmpLayerRect.rects.push_back(rect);
    }
    tmpVia.layerRects.push_back(tmpLayerRect);
  }
  
  int viaIdx = ((sproute_db::lefDataBase*) data)->vias.size();
  ((sproute_db::lefDataBase*) data)->lefVia2idx.insert( pair<string, int> (tmpVia.name, viaIdx));
  ((sproute_db::lefDataBase*) data)->vias.push_back(tmpVia);

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
  sproute_db::ViaRuleGenerate tmpViaRule;

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
  
  int viaRuleIdx = ((sproute_db::lefDataBase*) data)->viaRuleGenerates.size();
  ((sproute_db::lefDataBase*) data)->viaRuleGenerate2idx.insert( pair<string, int> (tmpViaRule.name, viaRuleIdx));
  ((sproute_db::lefDataBase*) data)->viaRuleGenerates.push_back(tmpViaRule);

  if(enableOutput)
    tmpViaRule.print();

  return 0;
}

}//namespace sproute_db

