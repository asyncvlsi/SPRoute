#include "sproute.h"
#include "algo.h"

#include "galois/Galois.h"
#include "galois/Reduction.h"
#include "galois/PriorityQueue.h"
#include "galois/Timer.h"
#include "galois/graphs/Graph.h"
#include "galois/graphs/TypeTraits.h"
#include "galois/substrate/SimpleLock.h"
#include "galois/AtomicHelpers.h"
#include "galois/runtime/Profile.h"

#include "galois/LargeArray.h"
#include "maze.h"

//#include "Lonestar/BFS_SSSP.h"

#include "DataType.h"
#include "flute.h"
#include "DataProc.h"
#include "RSMT.h"
#include "maze.h"
#include "RipUp.h"
#include "utility.h"
#include "route.h"
#include "maze3D.h"
#include "maze_finegrain.h"
#include "maze_finegrain_lateupdate.h"
#include "maze_lock.h"

#include "algo.h"
#include "grGen.h"
#include "detpart.h"
#include "a_star.h"
#include "invalidnets_adj.h"
#include "mempin_adj.h"
#include "rudy.h"
#include "detpart_astar.h"
#include "detpart_astar_data.h"
#include "detpart_astar_local.h"

namespace sproute {

void SPRoute::LinkTrackToLayer()
{
    for(auto layer : lefDB.layers)
    {
        if(layer.type == "ROUTING")
        {
            if(layer.direction == "HORIZONTAL")   // HORIZONTAL corresponds to Y location of a track
            {
                for(int i = 0; i < defDB.tracks.size(); i++)
                {
                    auto track = defDB.tracks.at(i);
                    for(auto layerName : track.layerNames)
                    {
                        if(layerName == layer.name && track.direction == "Y") {
                            defDB.layeridx2trackidx.insert( pair<int, int> (layer.idx, i));
                        }
                    }
                }
            }
            else if(layer.direction == "VERTICAL")
            {
                for(int i = 0; i < defDB.tracks.size(); i++)
                {
                    auto track = defDB.tracks.at(i);
                    for(auto layerName : track.layerNames)
                    {
                        if(layerName == layer.name && track.direction == "X") {
                            defDB.layeridx2trackidx.insert( pair<int, int> (layer.idx, i));
                        }
                    }
                }
            }
            else {
                cout << "error: unknown direction of lefDB layers" << endl;
                exit(1);
            }
        }
    }
}



void SPRoute::LoadPhyDB(phydb::PhyDB* p) {
    db_ptr_ = p;
    LoadPhyDBToLefDB();
    LoadPhyDBToDefDB();
}

void SPRoute::LoadPhyDBToLefDB() {
    lefDB.dbuPerMicron = db_ptr_->GetTechPtr()->GetDatabaseMicron();

    LoadPhyDBMacros();
    LoadPhyDBLefVias();
    LoadPhyDBLayers();
    //TODO: ADD VIARULEGENERATE
}

void SPRoute::LoadPhyDBMacros() {
    auto phydb_macros = db_ptr_->GetTechPtr()->GetMacrosRef();

    for(auto phydb_macro : phydb_macros) {
        sproute::Macro tmpMacro;
        int idx = lefDB.macros.size();
        tmpMacro.name = phydb_macro.GetName();
        tmpMacro.origin.x = phydb_macro.GetOriginX();
        tmpMacro.origin.y = phydb_macro.GetOriginY();
        tmpMacro.size.x = phydb_macro.GetWidth();
        tmpMacro.size.y = phydb_macro.GetHeight();

        auto phydb_pins = phydb_macro.GetPinsRef();
        for(auto phydb_pin : phydb_pins) {
            sproute::Pin tmpPin;
            tmpPin.name = phydb_pin.GetName();
            tmpPin.direction = phydb::SignalDirectionStr(phydb_pin.GetDirection());
            tmpPin.use = phydb::SignalUseStr(phydb_pin.GetUse());
            
            auto phydb_layer_rects = phydb_pin.GetLayerRectRef();
            for(auto phydb_layer_rect : phydb_layer_rects) {
                sproute::LayerRect tmpLayerRect;
                tmpLayerRect.layerName = phydb_layer_rect.layer_name_;
                for(auto phydb_rect : phydb_layer_rect.rects_) {
                    float lx = phydb_rect.LLX();
                    float ly = phydb_rect.LLY();
                    float ux = phydb_rect.URX();
                    float uy = phydb_rect.URY();
                    tmpLayerRect.rects.emplace_back(lx, ly, ux, uy);
                }
                tmpPin.layerRects.push_back(tmpLayerRect);
            }
            tmpMacro.pins.push_back(tmpPin);
        }


        auto phydb_layer_rects = phydb_macro.GetObs()->GetLayerRectsRef();
        for(auto phydb_layer_rect : phydb_layer_rects) {
            sproute::LayerRect tmpLayerRect;
            tmpLayerRect.layerName = phydb_layer_rect.layer_name_;
            for(auto phydb_rect : phydb_layer_rect.rects_) {
                float lx = phydb_rect.LLX();
                float ly = phydb_rect.LLY();
                float ux = phydb_rect.URX();
                float uy = phydb_rect.URY();
                tmpLayerRect.rects.emplace_back(lx, ly, ux, uy);
            }
            tmpMacro.obs.layerRects.push_back(tmpLayerRect);
        }
        lefDB.macros.push_back(tmpMacro);
        lefDB.macro2idx[tmpMacro.name] = idx;
    }
}


void SPRoute::LoadPhyDBLefVias() {
    auto phydb_vias = db_ptr_->GetTechPtr()->GetLefViasRef();

    for(auto phydb_via : phydb_vias) {
        sproute::lefVia tmpVia;
        int idx = lefDB.vias.size();
        tmpVia.name = phydb_via.GetName();

        auto phydb_layer_rects = phydb_via.GetLayerRectsRef();
        for(auto phydb_layer_rect : phydb_layer_rects) {
            sproute::LayerRect tmpLayerRect;
            tmpLayerRect.layerName = phydb_layer_rect.layer_name_;
            for(auto phydb_rect : phydb_layer_rect.rects_) {
                float lx = phydb_rect.LLX();
                float ly = phydb_rect.LLY();
                float ux = phydb_rect.URX();
                float uy = phydb_rect.URY();
                tmpLayerRect.rects.emplace_back(lx, ly, ux, uy);
            }
            tmpVia.layerRects.push_back(tmpLayerRect);
        }

        lefDB.vias.push_back(tmpVia);
        lefDB.lefVia2idx[tmpVia.name] = idx;
    }
}

void SPRoute::LoadPhyDBLayers() {
    auto phydb_layers = db_ptr_->GetTechPtr()->GetLayersRef();
    
    for(auto phydb_layer : phydb_layers) {
        sproute::Layer tmpLayer;
        int idx = lefDB.layers.size();
        tmpLayer.name = phydb_layer.GetName();
        tmpLayer.type = phydb::LayerTypeStr(phydb_layer.GetType());
        tmpLayer.idx = idx;
        if(phydb_layer.GetType() == phydb::LayerType::ROUTING) {
            tmpLayer.direction = phydb::MetalDirectionStr(phydb_layer.GetDirection());
            tmpLayer.pitchx = phydb_layer.GetPitchX();
            tmpLayer.pitchy = phydb_layer.GetPitchY();
            tmpLayer.width = phydb_layer.GetWidth();
            tmpLayer.area = phydb_layer.GetArea();
            tmpLayer.minWidth = phydb_layer.GetMinWidth();
            tmpLayer.offset = phydb_layer.GetOffset();

            tmpLayer.spacingTable.numCols = phydb_layer.GetSpacingTable()->GetNCol();
            tmpLayer.spacingTable.numRows = phydb_layer.GetSpacingTable()->GetNRow();

            auto prl_vec = phydb_layer.GetSpacingTable()->GetParallelRunLengthVec();
            auto width_vec = phydb_layer.GetSpacingTable()->GetWidthVec();
            auto spacing_vec = phydb_layer.GetSpacingTable()->GetSpacingVec();
            for(auto prl : prl_vec) {
                tmpLayer.spacingTable.parallelRunLength.push_back(prl);
            }

            for(auto width : width_vec) {
                tmpLayer.spacingTable.width.push_back(width);
            }

            for(auto spacing : spacing_vec) {
                tmpLayer.spacingTable.spacing.push_back(spacing);
            }

            auto phydb_eol_spacings_p = phydb_layer.GetEolSpacings();
            for(auto phydb_eol_spacing : *phydb_eol_spacings_p) {
                sproute::Spacing tmpSpacing;
                tmpSpacing.spacing = phydb_eol_spacing.GetSpacing();
                tmpSpacing.eolWidth = phydb_eol_spacing.GetEOLWidth();
                tmpSpacing.eolWithin = phydb_eol_spacing.GetEOLWithin();
                tmpSpacing.parEdge = phydb_eol_spacing.GetParEdge();
                tmpSpacing.parWithin = phydb_eol_spacing.GetParWithin();
                tmpLayer.spacings.push_back(tmpSpacing);
            }
        }
        else if(phydb_layer.GetType() == phydb::LayerType::CUT) {
            tmpLayer.spacing = phydb_layer.GetSpacing();
        }
        else {
            std::cout << "unknown phydb_layer type" << std::endl;
            exit(1);
        }

        lefDB.layers.push_back(tmpLayer);
        lefDB.layer2idx[tmpLayer.name] = idx;
    }
}

void SPRoute::LoadPhyDBDieArea() {
    auto phydb_die_area = db_ptr_->GetDesignPtr()->GetDieArea();
    int lx = phydb_die_area.LLX();
    int ly = phydb_die_area.LLY();
    int ux = phydb_die_area.URX();
    int uy = phydb_die_area.URY();

    defDB.dieArea.set(lx, ly, ux, uy);
}

void SPRoute::LoadPhyDBComponents() {
    auto phydb_components = db_ptr_->GetDesignPtr()->GetComponentsRef();

    for(auto phydb_component : phydb_components) {
        sproute::Component tmpComponent;
        int idx = defDB.components.size();

        tmpComponent.idx = idx;
        tmpComponent.name = phydb_component.GetName();
        tmpComponent.macroName = phydb_component.GetMacroName();
        tmpComponent.locationType = phydb::PlaceStatusStr(phydb_component.GetPlacementStatus());
        tmpComponent.orient = phydb::CompOrientStr(phydb_component.GetOrientation());
        tmpComponent.location.x = phydb_component.GetLocation().x;
        tmpComponent.location.y = phydb_component.GetLocation().y;
        tmpComponent.weight = phydb_component.GetWeight();

        defDB.components.push_back(tmpComponent);
        defDB.component2idx[tmpComponent.name] = idx;
    }

    defDB.numComponents = defDB.components.size();

}

void SPRoute::LoadPhyDBTracks() {
    auto phydb_tracks = db_ptr_->GetDesignPtr()->GetTracksRef();

    for(auto phydb_track : phydb_tracks) {
        sproute::Track tmpTrack;

        tmpTrack.direction = phydb::XYDirectionStr(phydb_track.GetDirection());
        tmpTrack.start = phydb_track.GetStart();
        tmpTrack.numTracks = phydb_track.GetNTracks();
        tmpTrack.step = phydb_track.GetStep();
        auto phydb_layer_names = phydb_track.GetLayerNames();
        for(auto phydb_layer_name : phydb_layer_names) {
            tmpTrack.layerNames.push_back(phydb_layer_name);
        }

        defDB.tracks.push_back(tmpTrack);
    }
}

void SPRoute::LoadPhyDBIOPins() {
    auto phydb_iopins = db_ptr_->GetDesignPtr()->GetIoPinsRef();

    for(auto phydb_iopin : phydb_iopins) {
        int idx = defDB.iopins.size();
        sproute::IOPin tmpIOPin;

        tmpIOPin.idx = idx;
        tmpIOPin.name = phydb_iopin.GetName();
        tmpIOPin.netName = phydb_iopin.GetNetName();
        tmpIOPin.direction = phydb::SignalDirectionStr(phydb_iopin.GetDirection());
        tmpIOPin.use = phydb::SignalUse(phydb_iopin.GetUse());
        tmpIOPin.layerName = phydb_iopin.GetLayerName();

        tmpIOPin.rect.set(phydb_iopin.GetRect().LLX(), phydb_iopin.GetRect().LLY(), phydb_iopin.GetRect().URX(), phydb_iopin.GetRect().URY());
        tmpIOPin.location.x = phydb_iopin.GetLocation().x;
        tmpIOPin.location.y = phydb_iopin.GetLocation().y;
        tmpIOPin.orient = phydb::CompOrientStr(phydb_iopin.GetOrientation());

        defDB.iopins.push_back(tmpIOPin);
        defDB.iopin2idx[tmpIOPin.name] = idx;
    }
    defDB.numIOPins = defDB.iopins.size();
}

void SPRoute::LoadPhyDBSNets() {
    auto phydb_snets = db_ptr_->GetDesignPtr()->GetSNetRef();

    for(auto phydb_snet : phydb_snets) {
        int idx = defDB.snets.size();
        sproute::SNet tmpSNet;

        tmpSNet.name = phydb_snet.GetName();
        auto phydb_paths = phydb_snet.GetPathRef();
        for(auto phydb_path : phydb_paths) {
            sproute::Path tmpPath;
            
            tmpPath.layerName = phydb_path.GetLayerName();
            tmpPath.width = phydb_path.GetWidth();
            tmpPath.shape = phydb_path.GetShape();
            tmpPath.viaName = phydb_path.GetViaName();
            tmpPath.beginExt = phydb_path.GetBeginExt();
            tmpPath.endExt = phydb_path.GetEndExt();
            tmpPath.rect.set(phydb_path.GetRect().LLX(), phydb_path.GetRect().LLX(), phydb_path.GetRect().LLX(), phydb_path.GetRect().LLX());
            tmpPath.begin.x = phydb_path.GetBegin().x;
            tmpPath.begin.y = phydb_path.GetBegin().y;
            tmpPath.begin.x = phydb_path.GetBegin().x;
            tmpPath.begin.y = phydb_path.GetBegin().y;

            tmpSNet.paths.push_back(tmpPath);
        }
        defDB.snets.push_back(tmpSNet);
    }
    defDB.numSpecialNets = defDB.snets.size();
}

void SPRoute::LoadPhyDBNets() {
    auto phydb_nets = db_ptr_->GetDesignPtr()->GetNetsRef();

    for(auto phydb_net : phydb_nets) {
        int idx = defDB.nets.size();
        sproute::Net tmpNet;

        tmpNet.name = phydb_net.GetName();
        tmpNet.use = phydb::SignalUse(phydb_net.use_);

        auto phydb_component_names = phydb_net.GetComponentNamesRef();
        auto phydb_pin_names = phydb_net.GetPinNamesRef();
        auto phydb_iopin_names = phydb_net.GetIoPinNamesRef();

        for(auto phydb_iopin_name : phydb_iopin_names) {
            tmpNet.componentNames.push_back("PIN");
            tmpNet.pinNames.push_back(phydb_iopin_name);
        }

        for(int i = 0; i < phydb_component_names.size(); i++) {
            tmpNet.componentNames.push_back(phydb_component_names[i]);
            tmpNet.pinNames.push_back(phydb_pin_names[i]);
        }

        defDB.nets.push_back(tmpNet);
        defDB.netName2netidx[tmpNet.name] = idx;
    }
    defDB.numNets = defDB.nets.size();
}

/*void SPRoute::LoadPhyDBDefVias() {

    
    auto phydb_defvias = db_ptr_->GetDesignPtr()->GetDefViasRef();

    for(auto phydb_defvia : phydb_defvias) {
        int idx = defDB.vias.size();
    }
}*/

void SPRoute::LoadPhyDBGcellGrids() {
    auto phydb_gcellgrids = db_ptr_->GetDesignPtr()->GetGcellGridsRef();

    for(auto phydb_gcellgrid : phydb_gcellgrids) {
        sproute::GcellGrid tmpGcellGrid;

        tmpGcellGrid.direction = phydb::XYDirectionStr(phydb_gcellgrid.GetDirection());
        tmpGcellGrid.start = phydb_gcellgrid.GetStart();
        tmpGcellGrid.numBoundaries = phydb_gcellgrid.GetNBoundaries();
        tmpGcellGrid.step = phydb_gcellgrid.GetStep();

        defDB.gcellGrids.push_back(tmpGcellGrid);
    }
}

void SPRoute::LoadPhyDBToDefDB() {
    defDB.dbuPerMicro = db_ptr_->GetDesignPtr()->GetUnitsDistanceMicrons();

    LoadPhyDBDieArea();
    LoadPhyDBComponents();
    LoadPhyDBTracks();
    LoadPhyDBIOPins();
    LoadPhyDBSNets();
    LoadPhyDBNets();

    //LoadPhyDBDefVias(); TODO: Add phydb APIs first
    LoadPhyDBGcellGrids();

}

void SPRoute::PreprocessSpacingTable()
{
    int dbuPerMicro = defDB.dbuPerMicro;
    for(auto& layer:lefDB.layers)
    {   
        //PRL spacing
        auto& spacingTable = layer.spacingTable;
        spacingTable.numCols = spacingTable.parallelRunLength.size();
        spacingTable.numRows = spacingTable.width.size();
        for(auto& length : spacingTable.parallelRunLength)
            length *= dbuPerMicro;

        for(auto& width : spacingTable.width)
            width *= dbuPerMicro;

        spacingTable.setSize(spacingTable.parallelRunLength.size(), spacingTable.width.size());
        for(auto& spacing : spacingTable.spacing)
        {
            spacing *= dbuPerMicro;
        }


        //corner spacing
        auto& cornerSpacing = layer.cornerSpacing;
        cornerSpacing.eolWidth *= dbuPerMicro;
        for(auto& width : cornerSpacing.width)
            width *= dbuPerMicro;

        for(auto& spacing : cornerSpacing.spacing)
            spacing *= dbuPerMicro;


        //EOL spacing
        for(auto& eolSpacing : layer.spacings)
        {
            eolSpacing.spacing *= dbuPerMicro;
            eolSpacing.eolWidth *= dbuPerMicro;
            eolSpacing.eolWithin *= dbuPerMicro;
            eolSpacing.parEdge *= dbuPerMicro;
            eolSpacing.parWithin *= dbuPerMicro;
            eolSpacing.cutWithin *= dbuPerMicro;
        }

        auto& maxEOLSpacing = layer.maxEOLSpacing;

        for(auto& eolSpacing : layer.spacings)
        {
            maxEOLSpacing.spacing = std::max(maxEOLSpacing.spacing, eolSpacing.spacing);
            maxEOLSpacing.eolWidth = std::max(maxEOLSpacing.eolWidth, eolSpacing.eolWidth);
            maxEOLSpacing.eolWithin = std::max(maxEOLSpacing.eolWithin, eolSpacing.eolWithin);
            maxEOLSpacing.parEdge = std::max(maxEOLSpacing.parEdge, eolSpacing.parEdge);
            maxEOLSpacing.parWithin = std::max(maxEOLSpacing.parWithin, eolSpacing.parWithin);
            maxEOLSpacing.cutWithin = std::max(maxEOLSpacing.cutWithin, eolSpacing.cutWithin);
        }
    }
}

void SPRoute::PreprocessComponent( )
{
    int dbuPerMicro = defDB.dbuPerMicro;
    defDB.origOBS.resize(lefDB.layers.size());
    defDB.designRuleOBS.resize(lefDB.layers.size());
    for(auto& component : defDB.components)
    {   
        assert(lefDB.macro2idx.find(component.macroName) != lefDB.macro2idx.end());

        int macroIdx = lefDB.macro2idx.find(component.macroName)->second;
        component.macro = lefDB.macros.at(macroIdx); //This is a copy

        component.macro.origin.x *= dbuPerMicro;
        component.macro.origin.y *= dbuPerMicro;

        component.macro.size.x *= dbuPerMicro;
        component.macro.size.y *= dbuPerMicro;

        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    rect.lowerLeft.x *= dbuPerMicro;
                    rect.lowerLeft.y *= dbuPerMicro;
                    rect.upperRight.x *= dbuPerMicro;
                    rect.upperRight.y *= dbuPerMicro;
                }
            }
        }


        for(auto& layerRect : component.macro.obs.layerRects)
        {
            for(auto& rect : layerRect.rects)
            {
                rect.lowerLeft.x *= dbuPerMicro;
                rect.lowerLeft.y *= dbuPerMicro;
                rect.upperRight.x *= dbuPerMicro;
                rect.upperRight.y *= dbuPerMicro;
            }
        }
    
        ComputePinLocation(component);
        ComputeOBSLocation(component);

        //push OBS in record
        for(auto pin : component.macro.pins)
        {
            if(pin.name == "VDD" || pin.name == "VSS") 
            {
                for(auto layerRect : pin.layerRects)
                {
                    int layer = lefDB.layer2idx.find(layerRect.layerName)->second;
                    if(layer > 0)
                    {
                        for(auto rect : layerRect.rects)
                        {
                            defDB.origOBS.at(layer).push_back(rect);
                            /*if(component.name == "inst15277")
                            {
                                cout << component.name << "/" << pin.name << endl;
                                cout << rect << endl;
                            }*/
                        }
                    }   
                }
            }
        }

        for(auto layerRect : component.macro.obs.layerRects)
        {
            int layer = lefDB.layer2idx.find(layerRect.layerName)->second;
            if(layer > 0)
            {   
                for(auto rect : layerRect.rects)
                {
                    defDB.origOBS.at(layer).push_back(rect);
                    //rect.print();
                }
            }
        }
    }

}

void SPRoute::UpdateGcellGrid(int xtrack_step, int ytrack_step)
{
    sproute::GcellGrid tmpGcellGrid_x, tmpGcellGrid_x1, tmpGcellGrid_y, tmpGcellGrid_y1;
    int xgcell_step = xtrack_step * 15;
    int ygcell_step = ytrack_step * 15;
    tmpGcellGrid_x.start = defDB.dieArea.lowerLeft.x;
    tmpGcellGrid_x.numBoundaries = (defDB.dieArea.upperRight.x - defDB.dieArea.lowerLeft.x) / xgcell_step + 1;
    tmpGcellGrid_x.step = xgcell_step;
    tmpGcellGrid_x.direction = "X" ;

    tmpGcellGrid_y.start = defDB.dieArea.lowerLeft.y;
    tmpGcellGrid_y.numBoundaries = (defDB.dieArea.upperRight.y - defDB.dieArea.lowerLeft.y) / ygcell_step + 1;
    tmpGcellGrid_y.step = ygcell_step;
    tmpGcellGrid_y.direction = "Y" ;

    tmpGcellGrid_x1.start = tmpGcellGrid_x.start + tmpGcellGrid_x.step * (tmpGcellGrid_x.numBoundaries - 1);
    tmpGcellGrid_x1.numBoundaries = 2;
    tmpGcellGrid_x1.step = defDB.dieArea.upperRight.x - tmpGcellGrid_x1.start;
    tmpGcellGrid_x1.direction = "X" ;

    tmpGcellGrid_y1.start = tmpGcellGrid_y.start + tmpGcellGrid_y.step * (tmpGcellGrid_y.numBoundaries - 1);
    tmpGcellGrid_y1.numBoundaries = 2;
    tmpGcellGrid_y1.step = defDB.dieArea.upperRight.y - tmpGcellGrid_y1.start;
    tmpGcellGrid_y1.direction = "Y" ;

    cout << "GCELLGRID TO BE ADDED: " << endl;
    cout << "GCELLGRID X " << tmpGcellGrid_x.start << " DO " << tmpGcellGrid_x.numBoundaries << " STEP " << tmpGcellGrid_x.step << endl;
    cout << "GCELLGRID X " << tmpGcellGrid_x1.start << " DO " << tmpGcellGrid_x1.numBoundaries << " STEP " << tmpGcellGrid_x1.step << endl;
    cout << "GCELLGRID Y " << tmpGcellGrid_y.start << " DO " << tmpGcellGrid_y.numBoundaries << " STEP " << tmpGcellGrid_y.step << endl;
    cout << "GCELLGRID Y " << tmpGcellGrid_y1.start << " DO " << tmpGcellGrid_y1.numBoundaries << " STEP " << tmpGcellGrid_y1.step << endl;

    defDB.gcellGrids.push_back(tmpGcellGrid_x);
    defDB.gcellGrids.push_back(tmpGcellGrid_x1);

    defDB.gcellGrids.push_back(tmpGcellGrid_y);
    defDB.gcellGrids.push_back(tmpGcellGrid_y1);
};

void SPRoute::GenerateGcellGrid()
{
    int xtrack_step = 0, ytrack_step = 0;
    string routing_layer_name;
    for(auto layer : lefDB.layers)
    {
        if(layer.type == "ROUTING")
        {
            cout << "layer for gcell: " << layer.name << endl;
            routing_layer_name = layer.name;
            for(auto track : defDB.tracks)
            {
                for(auto layerName : track.layerNames)
                {
                    if(layerName == layer.name && track.direction == "Y")
                        ytrack_step = track.step;

                    if(layerName == layer.name && track.direction == "X")
                        xtrack_step = track.step;
                }
            }
            break;
        }
    }
    cout << "for GcellGrid xtrack_step: " << xtrack_step << "ytrack_step: " << ytrack_step << endl;

    UpdateGcellGrid(xtrack_step, ytrack_step);
}

void SPRoute::InitGcell()
{
    bool enableOutput = false;

    std::list<int> tmpXGcellBoundaries, tmpYGcellBoundaries;
    Point2D<int> maxBoundaryCnt;
    maxBoundaryCnt.x = 0;
    maxBoundaryCnt.y = 0;
    if(defDB.gcellGrids.size() == 0)
        GenerateGcellGrid();
    
    for(auto gcellGrid : defDB.gcellGrids)
    {
        if(gcellGrid.direction == "X")
        {   
            if(gcellGrid.numBoundaries > maxBoundaryCnt.x)
            {
                maxBoundaryCnt.x = gcellGrid.numBoundaries;
                defDB.commonGcell.x = gcellGrid.step;
            }

            for(int i = 0; i < gcellGrid.numBoundaries; i++)
            {
                if(std::find(tmpXGcellBoundaries.begin(), tmpXGcellBoundaries.end(), gcellGrid.start + i*gcellGrid.step) == tmpXGcellBoundaries.end())
                    tmpXGcellBoundaries.push_back(gcellGrid.start + i*gcellGrid.step);
            }
        }
        else if(gcellGrid.direction == "Y")
        {
            if(gcellGrid.numBoundaries > maxBoundaryCnt.y)
            {
                maxBoundaryCnt.y = gcellGrid.numBoundaries;
                defDB.commonGcell.y = gcellGrid.step;
            }
            for(int i = 0; i < gcellGrid.numBoundaries; i++)
            {
                if(std::find(tmpYGcellBoundaries.begin(), tmpYGcellBoundaries.end(), gcellGrid.start + i*gcellGrid.step) == tmpYGcellBoundaries.end())
                    tmpYGcellBoundaries.push_back(gcellGrid.start + i*gcellGrid.step);
            }
        }
        else
        {
            cout << "unknown gcell direction: " << gcellGrid.direction << endl;
            exit(1);
        }
    }
    tmpXGcellBoundaries.sort();
    tmpYGcellBoundaries.sort();

    defDB.xGcellBoundaries.clear();
    defDB.yGcellBoundaries.clear();

    for(auto xBoundary : tmpXGcellBoundaries)
        defDB.xGcellBoundaries.push_back(xBoundary);

    for(auto yBoundary : tmpYGcellBoundaries)
        defDB.yGcellBoundaries.push_back(yBoundary);

    if(0)
    {
        cout << "xboundaries: " <<endl;
        int print_cnt = 0;
        for(auto boundary : defDB.xGcellBoundaries)
        {
            cout << boundary << " ";
            if(print_cnt % 10 == 0)
                cout << endl;
            print_cnt++;
        }
        cout << endl;
    
        cout << "yboundaries: " <<endl;
        print_cnt = 0;
        for(auto boundary : defDB.yGcellBoundaries)
        {
            cout << boundary << " ";
            if(print_cnt % 10 == 0)
                cout << endl;
            print_cnt++;
        }
        cout << endl;
    }

    
    int numRoutingLayers = 0;
    for(auto layer : lefDB.layers)
    {
        if(layer.type == "ROUTING")
        {
            numRoutingLayers++;
        }
    }

    defDB.gcellGridDim.x = defDB.xGcellBoundaries.size() - 1;
    defDB.gcellGridDim.y = defDB.yGcellBoundaries.size() - 1;
    defDB.gcellGridDim.z = numRoutingLayers;
    
    assert(defDB.gcellGridDim.x > 0 && defDB.gcellGridDim.y > 0 && defDB.gcellGridDim.z > 0);
    //cout << "here? " << defDB.gcellGridDim.x << " " <<  defDB.gcellGridDim.y << " " <<  defDB.gcellGridDim.z << endl;
    defDB.gcells.resize(defDB.gcellGridDim.x * defDB.gcellGridDim.y * defDB.gcellGridDim.z);
}

void SPRoute::PreprocessSNet()
{
    int dbuPerMicro = defDB.dbuPerMicro;
    for(auto snet : defDB.snets)
    {
        for(auto path : snet.paths)
        {
            if(path.viaName == "")
            {
                int layerIdx = lefDB.layer2idx.find(path.layerName)->second;
                if(layerIdx > 0)
                {
                    if(!path.rect.empty())
                    {
                        Rect2D<float> tmpRect;
                        tmpRect.set(path.rect.lowerLeft.x, path.rect.lowerLeft.y, path.rect.upperRight.x, path.rect.upperRight.y);
                        defDB.origOBS.at(layerIdx).push_back(tmpRect);
                    }
                    else
                    {
                        Rect2D<float> tmpRect;
                        int llx, lly, urx, ury;

                        if(path.begin.empty() || path.end.empty())
                        {
                            cout << " path missing begin or end node!" << endl;
                            path.begin.print();
                            path.end.print();
                            exit(1); 
                        }
                        
                        if(path.begin.y  == path.end.y)
                        {
                            llx = (path.begin.x < path.end.x)? path.begin.x : path.end.x;
                            urx = (path.begin.x > path.end.x)? path.begin.x : path.end.x;
                            lly = path.begin.y;
                            ury = path.begin.y;
                        }
                        else if(path.begin.x  == path.end.x)
                        {
                            llx = path.begin.x;
                            urx = path.begin.x;
                            lly = (path.begin.y < path.end.y)? path.begin.y : path.end.y;
                            ury = (path.begin.y > path.end.y)? path.begin.y : path.end.y;
                        }
                        else
                        {
                            cout << "ERROR: not a line? " << endl;
                            path.begin.print();
                            path.end.print();
                            exit(1);
                        }
                        llx = llx - path.width/2;
                        lly = lly - path.width/2;
                        urx = urx + path.width/2;
                        ury = ury + path.width/2;
                        /*llx = llx - path.width*2;
                        lly = lly - path.width*2;
                        urx = urx + path.width*2;
                        ury = ury + path.width*2;*/
                        tmpRect.set(llx, lly, urx, ury);

                        defDB.origOBS.at(layerIdx).push_back(tmpRect);

                    }
                }
            }
            else // is a Via
            {
                //if(path.begin.x == 31500 && path.begin.y == 263600)
                //    cout << "here" << endl;
                if(lefDB.lefVia2idx.count(path.viaName))
                {
                    int viaIdx = lefDB.lefVia2idx.find(path.viaName)->second;
                    lefVia via = lefDB.vias.at(viaIdx);
                    for(auto layerRect : via.layerRects)
                    {
                        int layerIdx = lefDB.layer2idx.find(layerRect.layerName)->second;
                        if(layerIdx == 0)
                            continue;
                        auto layer = lefDB.layers.at(layerIdx);
                        if(layer.type == "ROUTING")
                        {
                            for(auto rect : layerRect.rects)
                            {
                                Rect2D<float> tmpRect;
                                float llx = rect.lowerLeft.x * dbuPerMicro;
                                float lly = rect.lowerLeft.y * dbuPerMicro;
                                float urx = rect.upperRight.x * dbuPerMicro;
                                float ury = rect.upperRight.y * dbuPerMicro;
                                tmpRect.set(path.begin.x + llx, path.begin.y + lly, path.begin.x + urx, path.begin.y + ury);
                                defDB.origOBS.at(layerIdx).push_back(tmpRect);
                            }
                        }
                    }
                }
                else if(defDB.defVia2idx.count(path.viaName))
                {
                    int viaIdx = defDB.defVia2idx.find(path.viaName)->second;
                    defVia via = defDB.vias.at(viaIdx);
                    int xshift = ((float)via.numCutCols - 1 ) / 2 * via.cutSpacing.x + via.botEnc.x;
                    int yshift = ((float)via.numCutRows - 1 ) / 2 * via.cutSpacing.y + via.botEnc.y;

                    string botLayer = via.layers[0];
                    string topLayer = via.layers[2];

                    int botLayerIdx = lefDB.layer2idx.find(botLayer)->second;
                    int topLayerIdx = lefDB.layer2idx.find(topLayer)->second;

                    Rect2D<float> tmpRect;
                    tmpRect.set(path.begin.x - xshift, path.begin.y - yshift, path.begin.x + xshift, path.begin.y + yshift);
                    //tmpRect.set(path.begin.x - 2*xshift, path.begin.y - 2*yshift, path.begin.x + 2*xshift, path.begin.y + 2*yshift);

                    defDB.origOBS.at(botLayerIdx).push_back(tmpRect);
                    defDB.origOBS.at(topLayerIdx).push_back(tmpRect);


                }
                else
                {
                    cout << " unable to find via: " << path.viaName << endl;
                }
            }

        }
    }
}

void SPRoute::PreprocessDesignRule()
{
    int dbuPerMicro = defDB.dbuPerMicro;
    for(int i = 0; i <  lefDB.layers.size(); i++)
    {
        if(defDB.origOBS.at(i).size() == 0 || lefDB.layers.at(i).type != "ROUTING")
            continue;
        string layerName = lefDB.layers.at(i).name;

        cout << "design rule: " << layerName << endl;

        int trackIdx = defDB.layeridx2trackidx.find(i)->second;
        int trackStep = defDB.tracks.at(trackIdx).step;
        Layer& layer = lefDB.layers.at(i);

        for(int OBSIdx = 0; OBSIdx < defDB.origOBS.at(i).size(); OBSIdx++)
        {
            auto rect = defDB.origOBS.at(i).at(OBSIdx);
            
            int x_diff = rect.upperRight.x - rect.lowerLeft.x;
            int y_diff = rect.upperRight.y - rect.lowerLeft.y;
            int length = std::max(x_diff, y_diff);
            int width = std::min(x_diff, y_diff);

            if(width != lefDB.layers.at(i).width * dbuPerMicro) { //not a standard-width wire

                //width = width * 2; //double the width for power ground filler cells which only provide half of the width
                
                auto lengthIt = std::upper_bound(layer.spacingTable.parallelRunLength.begin(), layer.spacingTable.parallelRunLength.end(), length);
                int col = std::distance(layer.spacingTable.parallelRunLength.begin(), lengthIt) - 1;

                auto widthIt = std::upper_bound(layer.spacingTable.width.begin(), layer.spacingTable.width.end(), width);
                int row = std::distance(layer.spacingTable.width.begin(), widthIt) - 1;

                assert(col >= 0 && col < layer.spacingTable.parallelRunLength.size());
                assert(row >= 0 && row < layer.spacingTable.width.size());

                int PRL_expand = layer.spacingTable.getSpacing(col, row);
                int EOL_expand = 0;
                int COR_expand = 0;

                //consider EOL
                auto& maxEOLSpacing = layer.maxEOLSpacing;
                if(width < maxEOLSpacing.eolWidth)
                {
                    EOL_expand = std::max(maxEOLSpacing.spacing, maxEOLSpacing.parEdge);
                }


                //consider corner spacing
                auto& CornerSpacing = layer.cornerSpacing;
                if(width >= CornerSpacing.eolWidth && CornerSpacing.width.size() != 0)
                {
                    auto corIt = std::upper_bound(CornerSpacing.width.begin(), CornerSpacing.width.end(), width);
                    int row = std::distance(CornerSpacing.width.begin(), corIt) - 1;
                    COR_expand = CornerSpacing.spacing.at(row);
                }
                
                int expand = std::max(PRL_expand, EOL_expand);
                expand = std::max(expand, COR_expand);
                Rect2D<int> expandRect;
                expandRect.lowerLeft.x = rect.lowerLeft.x - expand;
                expandRect.lowerLeft.y = rect.lowerLeft.y - expand;
                expandRect.upperRight.x = rect.upperRight.x + expand;
                expandRect.upperRight.y = rect.upperRight.y + expand;
                defDB.designRuleOBS.at(i).push_back(expandRect);
            }
            else { //standard width wire
                Rect2D<int> intRect;
                intRect.lowerLeft.x = rect.lowerLeft.x;
                intRect.lowerLeft.y = rect.lowerLeft.y;
                intRect.upperRight.x = rect.upperRight.x;
                intRect.upperRight.y = rect.upperRight.y;
                defDB.designRuleOBS.at(i).push_back(intRect);
            }

            //cout << "rect: " << rect.lowerLeft.x << "," << rect.lowerLeft.y << " " << rect.upperRight.x << "," << rect.upperRight.y << " expand:" << expand << endl;
        }
    }
}

void SPRoute::AdjustGcellCap()
{
    bool debug = false;
    for(int i = 1; i <  lefDB.layers.size(); i++)
    {
        if(defDB.designRuleOBS.at(i).size() != 0 && lefDB.layers.at(i).type == "ROUTING")
        {
            cout << lefDB.layers.at(i).name << endl;
        
            for(int x = 0; x < defDB.xGcellBoundaries.size() - 2; x++)
            {
                //cout << "flow graph: " << x << endl;
                for(int y = 0; y < defDB.yGcellBoundaries.size() - 2; y++)
                {
                    //cout << "flow graph: " << x << " " << y << endl;
                    Gcell& gcell = defDB.getGcell(x, y, i/2);
                        
                    int next_x, next_y;
                    if(lefDB.layers.at(i).direction == "HORIZONTAL")
                    {
                        next_x = x + 1;
                        next_y = y;
                    }
                    else if(lefDB.layers.at(i).direction == "VERTICAL")
                    {
                        next_x = x;
                        next_y = y + 1;
                    }
                    else{
                        cout << "unknown layer direction: " << lefDB.layers.at(i).name << endl;
                        exit(1);
                    }
                    if(next_x > defDB.gcellGridDim.x || next_y > defDB.gcellGridDim.y)
                        continue;

                    Gcell& nextGcell = defDB.getGcell(next_x, next_y, i/2);

                    //if(debug)
                    //    cout << x << " " << y << endl;

                    if(gcell.obs_state == NO_OBS && nextGcell.obs_state == NO_OBS)
                        continue;

                    int numUsedTrack = 0;

                    if(gcell.obs_state == NO_OBS) { // if current gcell is empty, only consider next gcell
                        for(int trackIdx = 0; trackIdx < nextGcell.numTracks; trackIdx++) {
                            if(nextGcell.trackUsed[trackIdx])
                                numUsedTrack++;
                        }
                    }
                    else if(nextGcell.obs_state == NO_OBS) { // if next gcell is empty, only consider current gcell
                        for(int trackIdx = 0; trackIdx < gcell.numTracks; trackIdx++) {
                            if(gcell.trackUsed[trackIdx])
                                numUsedTrack++;
                        }
                    } 
                    else {  //consider both
                        for(int trackIdx = 0; trackIdx < gcell.numTracks; trackIdx++) {
                            if(gcell.trackUsed[trackIdx] || nextGcell.trackUsed[trackIdx])
                                numUsedTrack++;
                        }
                    }

                    CapReduction tmpCapReduction;
                    tmpCapReduction.x = x;
                    tmpCapReduction.y = y;
                    tmpCapReduction.z = i / 2;
                    tmpCapReduction.newCap = gcell.numTracks - numUsedTrack;
                    assert(tmpCapReduction.newCap >= 0);

                    capReductions.push(tmpCapReduction);
                }
            }
        }
    }
    
}


void SPRoute::PreprocessDesignRuleOBS()
{
    for(int i = 0; i <  lefDB.layers.size(); i++)
    {
        if(defDB.designRuleOBS.at(i).size() == 0 || lefDB.layers.at(i).type != "ROUTING")
            continue;
        string layerName = lefDB.layers.at(i).name;

        cout << layerName << ": " << defDB.designRuleOBS.at(i).size() << " obs" <<endl;

        int trackIdx = defDB.layeridx2trackidx.find(i)->second;
        int trackStep = defDB.tracks.at(trackIdx).step;

        for(int OBSIdx = 0; OBSIdx < defDB.designRuleOBS.at(i).size(); OBSIdx++)
        {
            auto rect = defDB.designRuleOBS.at(i).at(OBSIdx);

            //if(rect.lowerLeft.x == 1793920 && rect.lowerLeft.y == 1560150)
            //    cout << "ok i find this obs" << endl;

            int xGcell_start = 0;
            int yGcell_start = 0;
            int xGcell_end = 0;
            int yGcell_end = 0;

            xGcell_start = find_Gcell(rect.lowerLeft.x - trackStep, defDB.xGcellBoundaries);
            yGcell_start = find_Gcell(rect.lowerLeft.y - trackStep, defDB.yGcellBoundaries);
            xGcell_end = find_Gcell(rect.upperRight.x + trackStep, defDB.xGcellBoundaries);
            yGcell_end = find_Gcell(rect.upperRight.y + trackStep, defDB.yGcellBoundaries);

            //cout << xGcell_start << " " << yGcell_start << " " << xGcell_end << " " << yGcell_end << endl;

            //cout << " ========== " << endl;
            
            if(lefDB.layers.at(i).direction == "HORIZONTAL" && yGcell_end < defDB.gcellGridDim.y - 1)
                yGcell_end += 1;
            else if(lefDB.layers.at(i).direction == "VERTICAL" && xGcell_end < defDB.gcellGridDim.x - 1)
                xGcell_end += 1;
            else if(lefDB.layers.at(i).direction != "HORIZONTAL" && lefDB.layers.at(i).direction != "VERTICAL")
            {
                cout << "unknown direction!" << endl;
                exit(1);
            }

            for(int x = xGcell_start; x < xGcell_end; x++)
            {
                for(int y = yGcell_start; y < yGcell_end; y++)
                {
                    /*CapReduction tmpCapReduction;
                    tmpCapReduction.x = x;
                    tmpCapReduction.y = y;
                    tmpCapReduction.z = i / 2;*/
                    //cout << " OBS GRID: " << x << " " << y << endl;
                    //if(x == 10 && y == 189) 
                    //    cout << "10, 189 obs inserted: " << rect << endl;
                    GcellInsertOBS(x, y , i / 2, rect, OBSIdx); 
                    /*CapReduction tmpCapReduction;
                    tmpCapReduction.x = x;
                    tmpCapReduction.y = y;
                    tmpCapReduction.z = i / 2;
                    tmpCapReduction.newCap = 0;
                    defDB.capReductions.push_back(tmpCapReduction);*/
                }
                
            }
            
        }
    }
    
}

void SPRoute::GcellInsertOBS(int x, int y, int z, Rect2D<int> rect, int OBSIdx)
{
    bool debug = false;
    Gcell& gcell = defDB.getGcell(x, y, z);
    string layerName = lefDB.layers.at(z*2).name;
    string direction = lefDB.layers.at(z*2).direction;

    if(defDB.layeridx2trackidx.count(z*2) == 0)
    {
        cout << "unable to find layer tracks!: " << layerName << endl;
        exit(1);
    }
    

    int gcellLower, gcellUpper;
    if(direction == "HORIZONTAL")// HORIZONTAL LAYER
    {
        gcellLower = defDB.yGcellBoundaries.at(y);
        gcellUpper = defDB.yGcellBoundaries.at(y + 1);
    }
    else if(direction == "VERTICAL")
    {
        gcellLower = defDB.xGcellBoundaries.at(x);
        gcellUpper = defDB.xGcellBoundaries.at(x + 1);
    }
    else
    {
        cout << "unkown direction of layer : " << layerName << endl;
        exit(1);
    }

    int trackIdx = defDB.layeridx2trackidx.find(z*2)->second;
    Track track = defDB.tracks.at(trackIdx);
    int trackStart = track.start;
    int trackStep = track.step;

    if(debug && x == 10 && y == 189 && z == 1) {
        cout << "10, 189, 1 (metal2) : " << endl;
        Rect2D<float> tmpRect;
        tmpRect.lowerLeft.x = (float)rect.lowerLeft.x / 2000.0;
        tmpRect.lowerLeft.y = (float)rect.lowerLeft.y / 2000.0;
        tmpRect.upperRight.x = (float)rect.upperRight.x / 2000.0;
        tmpRect.upperRight.y = (float)rect.upperRight.y / 2000.0;
        cout << tmpRect << endl;
    }

    int gcellTrackStart = (gcellLower - trackStart) / trackStep + 1; // both are inclusive, i.e. local 0-14
    int gcellTrackEnd = (gcellUpper - trackStart) / trackStep;

    Range<int> gcellRange(gcellTrackStart, gcellTrackEnd);


    if(gcell.trackUsed == NULL)
    {
        gcell.numTracks = gcellTrackEnd - gcellTrackStart + 1;
        gcell.trackUsed = new bool [gcell.numTracks];
        for(int i = 0; i < gcell.numTracks; i++)
        {
            gcell.trackUsed[i] = false;
        }

    }

    int std_width = lefDB.layers.at(z*2).width * defDB.dbuPerMicro;
    int rect_width = (direction == "HORIZONTAL")? rect.upperRight.y - rect.lowerLeft.y : rect.upperRight.x - rect.lowerLeft.x;

    bool expand = (rect_width == std_width)? false : true;

    if(debug && x == 10 && y == 189 && z == 1) {
        cout << "expand: " << expand << endl;
    }

    Range<int> OBSTrackRange = GetTrackRange(z*2, rect, expand);

    Range<int> overlapRange = RangeIntersection(gcellRange, OBSTrackRange);
    int localStart = overlapRange.start - gcellTrackStart;
    int localEnd = overlapRange.end - gcellTrackStart;

    if(debug && x == 10 && y == 189 && z == 1) {
        cout << "localStart: " << localStart << " localEnd: " << localEnd << endl;
    }

    for(int i = localStart; i <= localEnd; i++)
    {
        gcell.trackUsed[i] = true;
    }

    gcell.OBSIDlist.push_back(OBSIdx);

    Point2D<int> GcellLowerLeft(defDB.xGcellBoundaries.at(x), defDB.yGcellBoundaries.at(y));
    Point2D<int> GcellUpperRight(defDB.xGcellBoundaries.at(x + 1), defDB.yGcellBoundaries.at(y + 1));

    if(rect.lowerLeft.x <= GcellLowerLeft.x && rect.lowerLeft.y <= GcellLowerLeft.y && rect.upperRight.x >= GcellUpperRight.x && rect.upperRight.y >= GcellUpperRight.y)
        gcell.obs_state = FULLYCOVERED;
    else
    {
        gcell.obs_state = HAS_OBS;
    }

}

Range<int> SPRoute::RangeIntersection(Range<int> r1, Range<int> r2)
{
    Range<int> tmp;
    tmp.start = (r1.start > r2.start)? r1.start : r2.start;
    tmp.end = (r1.end < r2.end)? r1.end : r2.end;
    if(tmp.start > tmp.end)
        return Range<int>(0, 0);
    else
        return tmp;
} 

Range<float> SPRoute::RangeIntersection(Range<float> r1, Range<float> r2)
{
    Range<float> tmp;
    tmp.start = (r1.start > r2.start)? r1.start : r2.start;
    tmp.end = (r1.end < r2.end)? r1.end : r2.end;
    if(tmp.start > tmp.end)
        return Range<float>(0, 0);
    else
        return tmp;
} 



void SPRoute::InitGrGen() {
        grGen.capReductions_p = &capReductions;
		grGen.minWidth = 1;
		grGen.minSpacing = 1;
		grGen.viaSpacing = 1;
		grGen.gcellBegin.set(defDB.xGcellBoundaries.front(), defDB.yGcellBoundaries.front());
		grGen.numAdjust = 0;

		int numLayers = 0;
		for(auto layer : lefDB.layers)
		{
			if(layer.type == "ROUTING")
				numLayers++;
		}

		grGen.grid.x = 0;
		grGen.grid.y = 0;
		grGen.grid.z = numLayers;
		int maxGcellCnt_x = 0, maxGcellCnt_y = 0;
		for(auto gcellGrid : defDB.gcellGrids)
	    {
	        if(gcellGrid.direction == "X")
	        {
	            grGen.grid.x += (gcellGrid.numBoundaries - 1);
	            if(gcellGrid.numBoundaries > maxGcellCnt_x) {
	            	maxGcellCnt_x = gcellGrid.numBoundaries;
	            	grGen.gcellSize.x = gcellGrid.step;
	            }
	        }
	        else if(gcellGrid.direction == "Y")
	        {
	        	grGen.grid.y += (gcellGrid.numBoundaries - 1);
	        	if(gcellGrid.numBoundaries > maxGcellCnt_y) {
	            	maxGcellCnt_y = gcellGrid.numBoundaries;
	            	grGen.gcellSize.y = gcellGrid.step;
	            }
	        }
	        else
	        {
	            cout << "unknown gcell direction: " << gcellGrid.direction << endl;
	            exit(1);
	        }
	    }
	    cout << grGen.grid.x << " " << grGen.grid.y << " " << grGen.grid.z << endl;

	    cout << "gcellbegin: " << grGen.gcellBegin.x << " " << grGen.gcellBegin.y <<endl;
	    cout << "gcellSize: " << grGen.gcellSize.x << " " << grGen.gcellSize.y <<endl;


	    for(auto layer : lefDB.layers)
		{
			if(layer.type == "ROUTING")
			{
				if(layer.direction == "HORIZONTAL")
				{
					for(auto track : defDB.tracks)
					{
						for(auto layerName : track.layerNames)
						{
							if(layerName == layer.name && track.direction == "Y") {
								if(grGen.hCap.size() != 0)
								{
									grGen.hCap.push_back(grGen.gcellSize.y / track.step);
									grGen.vCap.push_back(0);
								}
								else
								{
									grGen.hCap.push_back(0);
									grGen.vCap.push_back(0);
								}
							}
						}
					}
				}
				else if(layer.direction == "VERTICAL")
				{
					for(auto track : defDB.tracks)
					{
						for(auto layerName : track.layerNames)
						{
							if(layerName == layer.name && track.direction == "X") {
								if(grGen.vCap.size() != 0)
								{
									grGen.vCap.push_back(grGen.gcellSize.x / track.step);
									grGen.hCap.push_back(0);
								}
								else
								{
									grGen.vCap.push_back(0);
									grGen.hCap.push_back(0);
								}
							}
						}
					}
				}
				else {
					cout << "error: unknown direction of lefDB layers" << endl;
					exit(1);
				}
			}
		}

		cout << " vcap: " ;
		for(auto cap : grGen.vCap)
			cout << cap << " ";
		cout << endl;

		cout << " hcap: " ;
		for(auto cap : grGen.hCap)
			cout << cap << " ";
		cout << endl;



		int cnt = 0, cnt1=0;
		grGen.numNets = defDB.nets.size();

    grGen.grnets.resize(grGen.numNets);

		//for(auto net : defDB.nets)   //TODO: This should be parallelized
		//{
 
			cout << "enters here? " << endl;
     galois::do_all(
		 	galois::iterate((uint32_t) 0, (uint32_t)grGen.numNets),
				[&](uint32_t idx) {
			grNet tmpGRnet;
			auto net = defDB.nets[idx];

			tmpGRnet.name = net.name;
			tmpGRnet.idx = cnt;
			cnt++;
			int numPins = net.pinNames.size();
			if(numPins > 1000)
				cout << "large net: " << net.name << " " << numPins << endl;
			tmpGRnet.numPins = numPins;
			for(int i = 0; i < numPins; i++)
			{
				int x, y, z;
				string compName = net.componentNames.at(i);
				string pinName = net.pinNames.at(i);


				if(compName == "PIN") // pin is an IO pin
				{
					int IOPinIdx = defDB.iopin2idx.find(pinName)->second;
					IOPin iopin = defDB.iopins.at(IOPinIdx);

					x = iopin.location.x;
					y = iopin.location.y;
					int pinx = find_Gcell(x, defDB.xGcellBoundaries);
					int piny = find_Gcell(y, defDB.yGcellBoundaries);
					z = lefDB.layer2idx[iopin.layerName] / 2 + 1;
			        tmpGRnet.pinRegion.insert(Point3D<int>(pinx, piny, z));
                }
				else
				{
					int compIdx = defDB.component2idx.find(compName)->second;
					Component& component = defDB.components.at(compIdx);
					bool found = false;
					for(auto& pin : component.macro.pins)
					{
						if(pinName == pin.name)
						{
							x = pin.layerRects.at(0).rects.at(0).lowerLeft.x;
							y = pin.layerRects.at(0).rects.at(0).lowerLeft.y;
							string layerName = pin.layerRects.at(0).layerName;
							z = lefDB.layer2idx.find(layerName)->second / 2 + 1;
							found = true;

							for(auto layerRects : pin.layerRects)
							{
							    int layerIdx = lefDB.layer2idx[layerRects.layerName];	
                                if(lefDB.layers[layerIdx].type != "ROUTING")
                                    continue;
                                int pinRegionz = layerIdx / 2 + 1;
                                int trackIdx = defDB.layeridx2trackidx[layerIdx];
								int trackStep = defDB.tracks.at(trackIdx).step;
								for(auto rect: layerRects.rects)
								{
									int xmin = find_Gcell(rect.lowerLeft.x - trackStep, defDB.xGcellBoundaries);
            						int ymin = find_Gcell(rect.lowerLeft.y - trackStep, defDB.yGcellBoundaries);
            						int xmax = find_Gcell(rect.upperRight.x + trackStep, defDB.xGcellBoundaries);
            						int ymax = find_Gcell(rect.upperRight.y + trackStep, defDB.yGcellBoundaries);

                                    xmin = (xmin < 0)? 0 : xmin;
                                    ymin = (ymin < 0)? 0 : ymin;
                                    xmax = (xmax >= grGen.grid.x)? grGen.grid.x - 1: xmax;
                                    ymax = (ymax >= grGen.grid.y)? grGen.grid.y - 1: ymax;
            						//if(xmin != xmax || ymin != ymax)
            						//	cout << "Pin covers two gcells: " << net.name << " " << component.name << "/" << pin.name << endl;

									for(int pinRegionx = xmin; pinRegionx <= xmax; pinRegionx++)
									{
										for(int pinRegiony = ymin; pinRegiony <= ymax; pinRegiony++)
											tmpGRnet.pinRegion.insert(Point3D<int>(pinRegionx, pinRegiony, pinRegionz));
									}
								}
							}
							break;
						}
					}
					assert(found == true);
				}
				Point3D<int> grpin(x, y, z);
				tmpGRnet.pins.push_back(grpin);
				//cout << x << " " << y << " " << z << endl;
			}
			grGen.grnets[idx] = tmpGRnet;
      //  cnt1++;
			//grnets.push_back(tmpGRnet);
			//netName2grnetidx.insert( pair< string, int> (tmpGRnet.name, tmpGRnet.idx));

		//	tmpGRnet.pinRegion.clear();
			//tmpGRnet.clear();

		}, galois::steal());

		cout << "out here? " << endl;
		std::cout << "numnets: " << grGen.numNets << endl;
    cout << "cnt1: " << cnt1 << endl;



}

void SPRoute::RunFastRoute(string OutFileName, int maxMazeRound, Algo algo) {

    clock_t t1, t2, t3, t4;
    float gen_brk_Time, reading_Time, P1_Time, P2_Time, P3_Time, maze_Time, totalTime, congestionmap_time;
    int iter, last_totalOverflow, diff_totalOverflow, enlarge, ripup_threshold;
    int i, j,past_overflow,cur_overflow;
    int L_afterSTOP;
    int ESTEP1,CSTEP1, thStep1;
    int ESTEP2,CSTEP2, thStep2;
	int ESTEP3,CSTEP3, thStep3, tUsage, CSTEP4;
	int Ripvalue, LVIter, cost_step;
	int maxOverflow, past_cong, last_cong, finallength, numVia, ripupTH3D, newTH, healingTrigger;
	int updateType, minofl, minoflrnd, mazeRound, upType, cost_type, bmfl, bwcnt;
	bool goingLV, healingNeed, noADJ, extremeNeeded, needOUTPUT;

	int parts = 128;
	int max_nets_per_part = 64;
	galois::LargeArray<bool> done;
	float astar_weight = 1.0;
    done.allocateBlocked(numValidNets + 1);
    for (int i = 0; i < numValidNets; i++) {
      done[i] = false;
	}

    bool input, WriteOut;
    input=WriteOut=0;
    cout << " nthreads: " << numThreads << endl;
    if(OutFileName != "")
    {
    	needOUTPUT = true;
    	strcpy(routingFile, OutFileName.c_str());
    }

	LB=0.9;
	UB=1.3;


	SLOPE=5;
	THRESH_M=20;
	ENLARGE=115;//5
	ESTEP1=30;//10
	ESTEP2=30;//5
	ESTEP3=30;//5
	CSTEP1=2;//5
	CSTEP2=2;//3
	CSTEP3=5;//15
	CSTEP4 = 1000;
	COSHEIGHT=40;
	L=0;
	VIA=2;
	L_afterSTOP=1;
	Ripvalue=-1;
	ripupTH3D = 10;
	goingLV = TRUE;
	noADJ = FALSE;
	thStep1 = 10;
	thStep2 = 4;
	healingNeed = FALSE;
	updateType = 0;
	LVIter = 3;
	extremeNeeded = FALSE;
	mazeRound = maxMazeRound;
	bmfl = BIG_INT;
	minofl = BIG_INT;

     //galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE> thread_local_storage;
    galois::setActiveThreads(numThreads);
    galois::on_each( 
            [&] (const unsigned tid, const unsigned numT)
            {
                printf("threadid: %d total numT: %d\n", tid, numT);
            }
            );

   cout << " nthreads: " << numThreads << endl;

    
   int thread_choice = 0;
   int thread_steps[6] = {28,14,8,4,1};
   int thread_livelock_limit[6] = {1,1,1,1,1};
   bool extrarun = false;
   int thread_livelock = 0;

    if(1)
	{
		t1 = clock();
		printf("\nReading %s ...\n", benchFile);
		//readFile(benchFile);
		readGR(grGen, algo);

		printf("\nReading Lookup Table ...\n");
		readLUT();
		printf("\nDone reading table\n\n");  

		


		t2 = clock();
		reading_Time = (float)(t2-t1)/CLOCKS_PER_SEC;
		printf("Reading Time: %f sec\n", reading_Time);
	    
		// call FLUTE to generate RSMT and break the nets into segments (2-pin nets)

		VIA=2;
		//viacost = VIA;
		viacost = 0;
		gen_brk_RSMT(FALSE, FALSE, FALSE, FALSE, noADJ);
		printf("first L\n");
		routeLAll(TRUE);
        gen_brk_RSMT(FALSE, TRUE,TRUE,FALSE, noADJ);
		//gen_brk_RSMT(TRUE, TRUE,TRUE,FALSE, noADJ); // for the contest only
		getOverflow2D( &maxOverflow);
		printf("second L\n");
		newrouteLAll(FALSE, TRUE);
		getOverflow2D( &maxOverflow); 
		spiralRouteAll ();
		newrouteZAll(10) ;
		printf("first Z\n");
		past_cong = getOverflow2D( &maxOverflow); 

		convertToMazeroute();

		enlarge = 10;
		newTH = 10;
		healingTrigger = 0;
		stopDEC = 0;
		upType  = 1;

		//iniBDE();

		costheight=COSHEIGHT;

		if (maxOverflow > 700) {
			costheight = 8;
			LOGIS_COF = 1.33;
			VIA = 1;
			THRESH_M = 0;
			CSTEP1 = 30;
			slope = BIG_INT;
		}


		for (i = 0; i < LVIter; i++) {

			LOGIS_COF = max (2.0/(1+log(maxOverflow)), LOGIS_COF);
			LOGIS_COF = 2.0/(1+log(maxOverflow));
			printf("LV routing round %d, enlarge %d \n", i,enlarge);
			routeLVAll(newTH, enlarge);

			past_cong = getOverflow2Dmaze( &maxOverflow , & tUsage, false); 

			enlarge += 5;
			newTH -= 5;
			if (newTH < 1) {
				newTH = 1;
			}
		}  
	
		t3 = clock();
		reading_Time = (float)(t3-t2)/CLOCKS_PER_SEC;
		printf("LV Time: %f sec\n", reading_Time);
		InitEstUsage();

		i=1;
		costheight=COSHEIGHT;
		enlarge=ENLARGE;
		ripup_threshold=Ripvalue;
		
		minofl = past_cong;
		stopDEC = FALSE;

		slope = 20;
		L = 1;
		cost_type = 1;

		InitLastUsage(upType);
		
		PRINT_HEAT = 0;
		//checkUsageCorrectness();
		galois::StatTimer roundtimer("round");
		galois::StatTimer rudytimer("rudy");
		unsigned long oldtime = 0;
		round_avg_dist = 0;
		round_avg_length = 0;

		galois::substrate::PerThreadStorage<THREAD_LOCAL_STORAGE>
      		thread_local_storage{};

		while(totalOverflow>0)
		{
			/*if(THRESH_M>15) {
				THRESH_M-=thStep1;
			} else if(THRESH_M>=2) {
				THRESH_M-=thStep2;
			} else {
				THRESH_M = 0;
			}
			if(THRESH_M<=0) {
				THRESH_M=0;
			}*/
			THRESH_M = 0;
			//std::cout << "totalOverflow : " << totalOverflow << " enlarge: " << enlarge << std::endl; 
			if(totalOverflow>2000)
			{
				enlarge+=ESTEP1;//ENLARGE+(i-1)*ESTEP;
				cost_step = CSTEP1;
				updateCongestionHistory(upType);
				
			}
			else if(totalOverflow<500)
			{
				 
				cost_step = CSTEP3;
				enlarge+=ESTEP3;
				ripup_threshold = -1;
				updateCongestionHistory(upType);
			}  else	{
				cost_step = CSTEP2;
				enlarge+=ESTEP2;
				updateCongestionHistory(upType);
			}

			if(totalOverflow>15000 && maxOverflow > 400) {
				//enlarge = max(xGrid,yGrid) / 30;  //This is the key!!!! to enlarge routing area!!!!
				//enlarge = max(xGrid,yGrid) / 10;
				slope = BIG_INT;
				//slope = 20;
				if (i == 5) {
					VIA = 1;
					LOGIS_COF = 1.33;
					ripup_threshold = -1;
				//	cost_type = 3;
					
				} else if (i > 6) {
					if (i %2 == 0) {
						LOGIS_COF += 0.5;
					}
					if (i > 20){
						break;
					}
				} 
				if (i > 10) {
					cost_type = 1;
					ripup_threshold = 0;
				}
			}

			 
			enlarge = min (enlarge, max(xGrid, yGrid)/2);
			//std::cout << "costheight : " << costheight << " enlarge: " << enlarge << std::endl; 
			costheight+=cost_step;
			//std::cout << "costheight : " << costheight << " enlarge: " << enlarge << std::endl; 
			mazeedge_Threshold = THRESH_M;

			if (upType == 3) {
				LOGIS_COF = max (2.0/(1+log(maxOverflow+max_adj)),LOGIS_COF);
			} else {
				LOGIS_COF = max (2.0/(1+log(maxOverflow)),LOGIS_COF);
			}

			if (i == 8) {
				L = 0; 
				upType = 2;
				InitLastUsage(upType);
			} 

			 
			if (maxOverflow == 1) {
				//L = 0;
				ripup_threshold = -1;
				slope = 5;
			}
 
			if (maxOverflow > 300 && past_cong > 15000) {
				L = 0;
			}
			//checkUsageCorrectness();

			//getOverflow2Dmaze(&maxOverflow , & tUsage); 

			printf("iteration %d, enlarge %d, costheight %d, threshold %d via cost %d \nlog_coef %f, healingTrigger %d cost_step %d slope %d L %f cost_type %d updatetype %d OBIM delta %d\n",
				i,enlarge,costheight,mazeedge_Threshold, VIA,LOGIS_COF, healingTrigger, cost_step, slope, L ,cost_type, upType, max(OBIM_delta, (int)(costheight / (2*slope))));
			//L = 2; 
			roundtimer.start();
			//galois::runtime::profileVtune( [&] (void) {
                round_num = i;
				switch(algo) {
					case FineGrain: {
						printf("finegrain\n");
						mazeRouteMSMD_finegrain_spinlock(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i%3), cost_type);
						break;
					}
					case NonDet: {
						if(i == 1)
							undone_filter(done);
						mazeRouteMSMD(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, false /*!(i % 3)*/, cost_type, done, thread_local_storage);
						break;
					}
					case DetPart: {
						std::vector<std::vector<int>> vecParts(parts);
						int len       = numValidNets / (14) + 1;
						int len_parts = len / parts + 1;

						for (int i = 0; i < numValidNets; i++) {
							int i1 = i / len;
							int i2 = (i - i1 * len) / len_parts;
							vecParts[i2].push_back(i);
						}
						std::cout << "parts: " << parts << std::endl;
					
						mazeRouteMSMDDetPart(i, enlarge, costheight, ripup_threshold,
											mazeedge_Threshold, false /*!(i % 3)*/, cost_type, parts,
											vecParts, done);
						break;
					}
					case DetPart_Astar: {
						std::vector<std::vector<int>> vecParts(parts);
						int len       = numValidNets / (14) + 1;
						int len_parts = len / parts + 1;

						for (int i = 0; i < numValidNets; i++) {
							int i1 = i / len;
							int i2 = (i - i1 * len) / len_parts;
							vecParts[i2].push_back(i);
						}
						std::cout << "parts: " << parts << std::endl;
					
						mazeRouteMSMDDetPart_Astar(i, enlarge, costheight, ripup_threshold,
											mazeedge_Threshold, false, cost_type, parts,
											vecParts, done);
						break;
					}
					case DetPart_Astar_Data: {
						if(i == 1)
							undone_filter(done);
						int nets_per_part;
						if((numValidNets - acc_count) > parts * max_nets_per_part)
							nets_per_part = max_nets_per_part;
						else
							nets_per_part = max(2, (numValidNets - acc_count) / parts + 1);
						cout << "nets_per_part: "  << nets_per_part << endl;
						std::vector<int> part;
						
						std::vector<std::vector<int>> vecParts;
						

						for (int netID = 0; netID < numValidNets; netID ++) {
							if(!done[netID]) {
								if(part.size() < nets_per_part) {
									part.push_back(netID);
								}
								else {
									vecParts.push_back(part);
									part.clear();
									part.push_back(netID);
								}
							}
						}
						
						if(part.size() != 0)
							vecParts.push_back(part);
						std::cout << "parts: " << vecParts.size() << std::endl;
					
						mazeRouteMSMDDetPart_Astar_Data(i, enlarge, costheight, ripup_threshold,
											mazeedge_Threshold, false, cost_type, vecParts.size(),
											vecParts, done, thread_local_storage);
						break;
					}
					case DetPart_Astar_Local: {
						if(i == 1)
							undone_filter(done);
						int nets_per_part;
						if((numValidNets - acc_count) > parts * max_nets_per_part)
							nets_per_part = max_nets_per_part;
						else
							nets_per_part = max(8, (numValidNets - acc_count) / parts + 1);

						int counted_num_part = (numValidNets - acc_count) / nets_per_part + 1;
						cout << "avg nets_per_part: "  << nets_per_part << " counted parts: " << counted_num_part << endl;
						std::vector<int> part;
						
						std::vector<std::vector<int>> vecParts;
						//vecParts.resize(counted_num_part)
						if(past_cong > 20000 && max_nets_per_part>= 32)
							max_nets_per_part /= 2;
						
						rudytimer.start();
						vecParts.resize(counted_num_part);
						/*for (int netID = 0; netID < numValidNets; netID ++) {
							if(!done[netID]) {
								vecParts[netID % counted_num_part].push_back(netID);
							}
						}
						for(int batchID = 0; batchID < vecParts.size(); batchID++) {
							cout << vecParts[batchID].size() << " ";
						}*/
						cout << endl;
						rudytimer.stop();
						for (int netID = 0; netID < numValidNets; netID ++) {
							if(!done[netID]) {
								if(part.size() < nets_per_part) {
									part.push_back(netID);
								}
								else {
									vecParts.push_back(part);
									part.clear();
									part.push_back(netID);
								}
							}
						}

						/*else {
							rudytimer.start();
							RUDY_scheduler(i, vecParts, done);
							rudytimer.stop();
						}*/
						
						if(part.size() != 0)
							vecParts.push_back(part);
						std::cout << "parts: " << vecParts.size() << std::endl;
					
						mazeRouteMSMDDetPart_Astar_Local(i, enlarge, costheight, ripup_threshold,
											mazeedge_Threshold, false, cost_type, vecParts.size(),
											vecParts, done, thread_local_storage);
						break;
					}
					
					case DetPart_Astar_RUDY: {
						if(i == 1)
							undone_filter(done);
						int nets_per_part;
						if((numValidNets - acc_count) > parts * max_nets_per_part)
							nets_per_part = max_nets_per_part;
						else
							nets_per_part = max(8, (numValidNets - acc_count) / parts + 1);

						int counted_num_part = (numValidNets - acc_count) / nets_per_part + 1;
						
						std::vector<std::vector<int>> vecParts;
						
						//if(i >= 2)
						rudytimer.start();
						if(i == 1) {
							vecParts.resize(counted_num_part);
							for (int netID = 0; netID < numValidNets; netID ++) {
								if(!done[netID]) {
									vecParts[netID % counted_num_part].push_back(netID);
								}
							}
							for(int batchID = 0; batchID < vecParts.size(); batchID++) {
								cout << vecParts[batchID].size() << " ";
							}
							cout << endl;
						}
						else {
							RUDY_scheduler(i, maxOverflow, counted_num_part, vecParts, done);
						}
						rudytimer.stop();
						/*else {
							for (int netID = 0; netID < numValidNets; netID ++) {
								if(!done[netID]) {
									if(part.size() < nets_per_part) {
										part.push_back(netID);
									}
									else {
										vecParts.push_back(part);
										part.clear();
										part.push_back(netID);
									}
								}
							}
						}*/
						
						std::cout << "parts: " << vecParts.size() << std::endl;
					
						mazeRouteMSMDDetPart_Astar_Local(i, enlarge, costheight, ripup_threshold,
											mazeedge_Threshold, false, cost_type, vecParts.size(),
											vecParts, done, thread_local_storage);
						break;
					}
					case Astar: {
						mazeRouteMSMD_astar(i,enlarge, costheight, ripup_threshold,mazeedge_Threshold, !(i % 3), cost_type, astar_weight, done);
						break;
					}
					default: {
						cout << " unkown algo: " << algo << endl;
						exit(1);
					}
				}
				
			//}, "mazeroute");
			roundtimer.stop();
			cout << "round : " << i << " time(ms): " << roundtimer.get() - oldtime << " acc time(ms): " << roundtimer.get() << " scheduling time: " << rudytimer.get()<< endl;
			oldtime = roundtimer.get();
			//checkUsageCorrectness();
            last_cong = past_cong;
 
			past_cong = getOverflow2Dmaze(&maxOverflow , & tUsage); 
			//if(i == 1)
			//	break;
            int nthreads_tmp = numThreads;
			if(algo == NonDet && past_cong > last_cong && !extrarun)  // Michael
			{ 
				if(algo != FineGrain && nthreads_tmp != 1)
				{
					thread_livelock++;
					if(thread_livelock == 1)
					{
						thread_choice++;
						thread_livelock = 0;
                        if(nthreads_tmp < 6) {
						    galois::setActiveThreads(2);
                            numThreads = 2;
                            //algo = FineGrain;
                        }
                        else {
                            numThreads = numThreads / 2;
							galois::setActiveThreads(numThreads);
					    }
                    }
				}
			}

			
			cout << "nthreads :" <<	numThreads << endl;
			extrarun = false;

			if (minofl > past_cong) {
				minofl =  past_cong;
				minoflrnd = i;
				if(i > 5)
					copyRS();
			}
			//if(i == 2) break;
			if(i == 4 && minofl > 5000)
				break; 

			if (i == 8) {
				L = 1;
			}

			i++;

			if (past_cong < 200 && i > 30 && upType == 2 && max_adj <=20) {
				upType = 4;
				stopDEC = TRUE;
			}
			
			if (i > 50 ) {
				upType = 4;
				if (i > 70) {
					stopDEC = TRUE;
				}
			}
				
			if (past_cong > 0.7 * last_cong) {
				costheight += CSTEP3;
			}


			if (past_cong >= last_cong ) {
				//VIA = 0; //is this good?
				healingTrigger ++;
			} 
			
			if (past_cong < bmfl) {
				bwcnt = 0;			
			} else {
				bwcnt ++;
			}

			if (bmfl > 10) {
				if (bmfl > 30 && bmfl < 72 && bwcnt > 50) {
					break;
				}
				if (bmfl < 30 && bwcnt > 50) {
					break;
				}
				if (i >= mazeRound) {
					getOverflow2Dmaze( &maxOverflow, & tUsage);
					break;
				}
			}


			if (i >= mazeRound) {
				getOverflow2Dmaze( &maxOverflow, & tUsage);
				break;
			}
		}
		
		if (minofl > 0) {
			printf("\n\n minimal ofl %d, occuring at round %d\n\n",minofl,minoflrnd);
			copyBR();
		} 
			
		freeRR();
		

		checkUsage();

		printf("maze routing finished\n");
         
		t4 = clock();
		maze_Time = (float)(t4-t3)/CLOCKS_PER_SEC;
		//printf("P3 runtime: %f sec\n", maze_Time);

		printf("Final 2D results: \n");
		getOverflow2Dmaze( &maxOverflow , & tUsage);

		printf("\nLayer Assignment Begins");
		newLA ();
		printf("layer assignment finished\n");

		t2 = clock();
		gen_brk_Time = (float)(t2-t1)/CLOCKS_PER_SEC;
		//printf("2D + Layer Assignment Runtime: %f sec\n", gen_brk_Time); 

		costheight = 3;
		viacost = 1;

		if (gen_brk_Time < 60) {
			ripupTH3D = 15;
		} else if (gen_brk_Time < 120) {
			ripupTH3D = 18;
		} else {
			ripupTH3D = 20;
		}

		

		if (goingLV && past_cong == 0) {
			printf("Post Processing Begins \n");
			//mazeRouteMSMDOrder3D(enlarge, 0, ripupTH3D );
			
		//	mazeRouteMSMDOrder3D(enlarge, 0, 10 );
			if (gen_brk_Time > 120) {
				//mazeRouteMSMDOrder3D(enlarge, 0, 12 );
			}
			printf("Post Processsing finished, starting via filling\n");			

		}
		
		fillVIA();
		finallength = getOverflow3D();
		numVia= threeDVIA ();
		checkRoute3D();
		//printUndone(done);
		
		/*if (needOUTPUT) {
			string overflowFile = "overflow.guide";
			writeOverflow(overflowFile, grGen);
		}*/
		
	}//Input ==1



	t4 = clock();
	maze_Time = (float)(t4-t1)/CLOCKS_PER_SEC;
	printf("Final routing length : %d\n",finallength);
	printf("Final number of via  : %d\n",numVia);
	printf("Final total length 1 : %d\n\n",finallength+numVia);
    	
    //printf("Final total length 3 : %d\n",(finallength+3*numVia));
	//printf("3D runtime: %f sec\n", maze_Time);

	//freeAllMemory();
	return ;
}

void SPRoute::ComputePinLocation(sproute::Component& component)
{
    int llx, lly, urx, ury;
    if(component.orient == "N")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + component.macro.origin.x + rect.lowerLeft.x;
                    lly = component.location.y + component.macro.origin.y + rect.lowerLeft.y;
                    urx = component.location.x + component.macro.origin.x + rect.upperRight.x;
                    ury = component.location.y + component.macro.origin.y + rect.upperRight.y;
                    rect.set(llx, lly, urx, ury);
                }
            }
        }
    }
    else if(component.orient == "S")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + component.macro.origin.x + component.macro.size.x - rect.upperRight.x;
                    lly = component.location.y + component.macro.origin.y + component.macro.size.y - rect.upperRight.y;
                    urx = component.location.x + component.macro.origin.x + component.macro.size.x - rect.lowerLeft.x;
                    ury = component.location.y + component.macro.origin.y + component.macro.size.y - rect.lowerLeft.y;
                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else if(component.orient == "E")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime =  centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(270deg)centerx - sin(270deg)centery
                    float centeryprime = -centerx; 
                    // sin(270deg)centerx + cos(270deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 270deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = centerxprime - rectwidth/2.0;
                    lly = centeryprime - rectheight/2.0;
                    urx = centerxprime + rectwidth/2.0;
                    ury = centeryprime + rectheight/2.0;
                    // shift upward for a macro height, now we should use macro.size.x, after shift move to (comp.placeLoc.x, comp.placeLoc.y)
                    llx += component.location.x;
                    lly += component.macro.size.x + component.location.y;
                    urx += component.location.x;
                    ury += component.macro.size.x + component.location.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else if(component.orient == "W")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime = -centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(90deg)centerx - sin(90deg)centery
                    float centeryprime =  centerx; 
                    // sin(90deg)centerx + cos(90deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 90deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = std::round(centerxprime - rectwidth/2.0);
                    lly = std::round(centeryprime - rectheight/2.0);
                    urx = std::round(centerxprime + rectwidth/2.0);
                    ury = std::round(centeryprime + rectheight/2.0);
                    // shift rightward for a macro width, now we should use macro.size.y, after shift move to (comp.placeLoc.x, comp.placeLoc.y)
                    llx += component.macro.size.y + component.location.x;
                    lly += component.location.y;
                    urx += component.macro.size.y + component.location.x;
                    ury += component.location.y;


                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else if(component.orient == "FN")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + component.macro.size.x - rect.upperRight.x;
                    lly = component.location.y + rect.lowerLeft.y;
                    urx = component.location.x + component.macro.size.x - rect.lowerLeft.x;
                    ury = component.location.y + rect.upperRight.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else if(component.orient == "FS")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + rect.lowerLeft.x;
                    lly = component.location.y + component.macro.size.y - rect.upperRight.y;
                    urx = component.location.x + rect.upperRight.x;
                    ury = component.location.y + component.macro.size.y - rect.lowerLeft.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else if(component.orient == "FE")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime =  centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(270deg)centerx - sin(270deg)centery
                    float centeryprime = -centerx; 
                    // sin(270deg)centerx + cos(270deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 270deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = centerxprime - rectwidth/2.0;
                    lly = centeryprime - rectheight/2.0;
                    urx = centerxprime + rectwidth/2.0;
                    ury = centeryprime + rectheight/2.0;
                    // shift upward for a macro height, now we should use macro.size.x
                    lly += component.macro.size.x;
                    ury += component.macro.size.x;

                    int axis = component.macro.size.y;
                    centerx = (llx + urx)/2.0;
                    centery = (lly + ury)/2.0;
                    rectwidth = urx - llx;
                    rectheight = ury - lly;

                    centerx = axis - centerx;

                    llx = std::round(centerx - rectwidth/2.0) + component.location.x;
                    lly = std::round(centery - rectheight/2.0) + component.location.y;
                    urx = std::round(centerx + rectwidth/2.0) + component.location.x;
                    ury = std::round(centery + rectheight/2.0) + component.location.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;

                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else if(component.orient == "FW")
    {
        for(auto& pin : component.macro.pins)
        {
            for(auto& layerRect : pin.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime = -centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(90deg)centerx - sin(90deg)centery
                    float centeryprime =  centerx; 
                    // sin(90deg)centerx + cos(90deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 90deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = std::round(centerxprime - rectwidth/2.0);
                    lly = std::round(centeryprime - rectheight/2.0);
                    urx = std::round(centerxprime + rectwidth/2.0);
                    ury = std::round(centeryprime + rectheight/2.0);
                    // shift rightward for a macro width, now we should use macro.size.y
                    llx += component.macro.size.y;
                    urx += component.macro.size.y;
                    // flip, this is done using the right boundary location, which is macro.size.y
                    int axis = component.macro.size.y;
                    centerx = (llx + urx)/2.0;
                    centery = (lly + ury)/2.0;
                    rectwidth = urx - llx;
                    rectheight = ury - lly;

                    centerx = axis - centerx;

                    llx = std::round(centerx - rectwidth/2.0) + component.location.x;
                    lly = std::round(centery - rectheight/2.0) + component.location.y;
                    urx = std::round(centerx + rectwidth/2.0) + component.location.x;
                    ury = std::round(centery + rectheight/2.0) + component.location.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;

                    rect.set(llx, lly, urx, ury);
                }
            }
        }

    }
    else
    {
        cout << "unknown orientation for component: " << component.idx << " " << component.name << " " << component.orient << endl;
        exit(1);
    }
}


void SPRoute::ComputeOBSLocation(sproute::Component& component)
{
    int llx, lly, urx, ury;
    if(component.orient == "N")
    {
        for(auto& layerRect : component.macro.obs.layerRects)
        {
            for(auto& rect : layerRect.rects)
            {
                llx = component.location.x + component.macro.origin.x + rect.lowerLeft.x;
                lly = component.location.y + component.macro.origin.y + rect.lowerLeft.y;
                urx = component.location.x + component.macro.origin.x + rect.upperRight.x;
                ury = component.location.y + component.macro.origin.y + rect.upperRight.y;
                rect.set(llx, lly, urx, ury);
            }
        }
    }
    else if(component.orient == "S")
    {

            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + component.macro.origin.x + component.macro.size.x - rect.upperRight.x;
                    lly = component.location.y + component.macro.origin.y + component.macro.size.y - rect.upperRight.y;
                    urx = component.location.x + component.macro.origin.x + component.macro.size.x - rect.lowerLeft.x;
                    ury = component.location.y + component.macro.origin.y + component.macro.size.y - rect.lowerLeft.y;
                    rect.set(llx, lly, urx, ury);
                }
            }

    }
    else if(component.orient == "E")
    {
            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime =  centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(270deg)centerx - sin(270deg)centery
                    float centeryprime = -centerx; 
                    // sin(270deg)centerx + cos(270deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 270deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = centerxprime - rectwidth/2.0;
                    lly = centeryprime - rectheight/2.0;
                    urx = centerxprime + rectwidth/2.0;
                    ury = centeryprime + rectheight/2.0;
                    // shift upward for a macro height, now we should use macro.size.x, after shift move to (comp.placeLoc.x, comp.placeLoc.y)
                    llx += component.location.x;
                    lly += component.macro.size.x + component.location.y;
                    urx += component.location.x;
                    ury += component.macro.size.x + component.location.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }

    }
    else if(component.orient == "W")
    {
            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime = -centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(90deg)centerx - sin(90deg)centery
                    float centeryprime =  centerx; 
                    // sin(90deg)centerx + cos(90deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 90deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = std::round(centerxprime - rectwidth/2.0);
                    lly = std::round(centeryprime - rectheight/2.0);
                    urx = std::round(centerxprime + rectwidth/2.0);
                    ury = std::round(centeryprime + rectheight/2.0);
                    // shift rightward for a macro width, now we should use macro.size.y, after shift move to (comp.placeLoc.x, comp.placeLoc.y)
                    llx += component.macro.size.y + component.location.x;
                    lly += component.location.y;
                    urx += component.macro.size.y + component.location.x;
                    ury += component.location.y;


                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }

    }
    else if(component.orient == "FN")
    {
            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + component.macro.size.x - rect.upperRight.x;
                    lly = component.location.y + rect.lowerLeft.y;
                    urx = component.location.x + component.macro.size.x - rect.lowerLeft.x;
                    ury = component.location.y + rect.upperRight.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }

    }
    else if(component.orient == "FS")
    {
            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    llx = component.location.x + rect.lowerLeft.x;
                    lly = component.location.y + component.macro.size.y - rect.upperRight.y;
                    urx = component.location.x + rect.upperRight.x;
                    ury = component.location.y + component.macro.size.y - rect.lowerLeft.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;
                    rect.set(llx, lly, urx, ury);
                }
            }

    }
    else if(component.orient == "FE")
    {
            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime =  centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(270deg)centerx - sin(270deg)centery
                    float centeryprime = -centerx; 
                    // sin(270deg)centerx + cos(270deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 270deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = centerxprime - rectwidth/2.0;
                    lly = centeryprime - rectheight/2.0;
                    urx = centerxprime + rectwidth/2.0;
                    ury = centeryprime + rectheight/2.0;
                    // shift upward for a macro height, now we should use macro.size.x
                    lly += component.macro.size.x;
                    ury += component.macro.size.x;

                    int axis = component.macro.size.y;
                    centerx = (llx + urx)/2.0;
                    centery = (lly + ury)/2.0;
                    rectwidth = urx - llx;
                    rectheight = ury - lly;

                    centerx = axis - centerx;

                    llx = std::round(centerx - rectwidth/2.0) + component.location.x;
                    lly = std::round(centery - rectheight/2.0) + component.location.y;
                    urx = std::round(centerx + rectwidth/2.0) + component.location.x;
                    ury = std::round(centery + rectheight/2.0) + component.location.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;

                    rect.set(llx, lly, urx, ury);
                }
            }
    }
    else if(component.orient == "FW")
    {
            for(auto& layerRect : component.macro.obs.layerRects)
            {
                for(auto& rect : layerRect.rects)
                {
                    float centerx = (rect.lowerLeft.x + rect.upperRight.x)/2.0;
                    float centery = (rect.lowerLeft.y + rect.upperRight.y)/2.0;
                    int rectwidth = rect.upperRight.x - rect.lowerLeft.x;
                    int rectheight = rect.upperRight.y - rect.lowerLeft.y;
                    float centerxprime = -centery; 
                    // assumption origin.x and origin.y is always 0
                    // cos(90deg)centerx - sin(90deg)centery
                    float centeryprime =  centerx; 
                    // sin(90deg)centerx + cos(90deg)centery
                    int tmplength;
                    tmplength = rectwidth; 
                    // 90deg rotation, width and height swap
                    rectwidth = rectheight;
                    rectheight = tmplength; 
                    // macro width and height should also swap, but we cannot do it here, because we are at pin level now
                    // but remember, when we need macro width, we should call its height, and vice versa
                    llx = std::round(centerxprime - rectwidth/2.0);
                    lly = std::round(centeryprime - rectheight/2.0);
                    urx = std::round(centerxprime + rectwidth/2.0);
                    ury = std::round(centeryprime + rectheight/2.0);
                    // shift rightward for a macro width, now we should use macro.size.y
                    llx += component.macro.size.y;
                    urx += component.macro.size.y;
                    // flip, this is done using the right boundary location, which is macro.size.y
                    int axis = component.macro.size.y;
                    centerx = (llx + urx)/2.0;
                    centery = (lly + ury)/2.0;
                    rectwidth = urx - llx;
                    rectheight = ury - lly;

                    centerx = axis - centerx;

                    llx = std::round(centerx - rectwidth/2.0) + component.location.x;
                    lly = std::round(centery - rectheight/2.0) + component.location.y;
                    urx = std::round(centerx + rectwidth/2.0) + component.location.x;
                    ury = std::round(centery + rectheight/2.0) + component.location.y;

                    llx += component.macro.origin.x;
                    lly += component.macro.origin.y;
                    urx += component.macro.origin.x;
                    ury += component.macro.origin.y;

                    rect.set(llx, lly, urx, ury);
                }
            }

    }
    else
    {
        cout << "unknown orientation for component: " << component.idx << " " << component.name << " " << component.orient << endl;
        exit(1);
    }
}

Range<int> SPRoute::GetTrackRange(int layerIdx, Rect2D<float> rect, bool expand)
{
    int lower, upper, startTrackIdx, endTrackIdx;
    int trackStart, trackStep;
    string direction = lefDB.layers.at(layerIdx).direction;
    string layerName = lefDB.layers.at(layerIdx).name;

    if(defDB.layeridx2trackidx.count(layerIdx) == 0)
    {
        cout << "unable to find layer in layer tracks: " << layerName << endl;
        exit(1);
    }

    if(direction == "HORIZONTAL") // for pessimism, disable one more track 
    {
        lower = rect.lowerLeft.y;
        upper = rect.upperRight.y;  
    }
    else if(direction == "VERTICAL")
    {
        lower = rect.lowerLeft.x;
        upper = rect.upperRight.x;
    }
    else
    {
        cout << "unkown direction of layer : " << layerIdx << endl;
        exit(1);
    }

    int trackIdx = defDB.layeridx2trackidx.find(layerIdx)->second;
    trackStart = defDB.tracks.at(trackIdx).start;
    trackStep = defDB.tracks.at(trackIdx).step;

    if(expand)
    {
	    startTrackIdx = (lower - trackStep - trackStart) / trackStep;
	    startTrackIdx = (startTrackIdx < 0)? 0 : startTrackIdx;
	    endTrackIdx = (upper + trackStep - trackStart) / trackStep;
	}
	else
	{
		startTrackIdx = (lower - trackStart) / trackStep + 1;
	    endTrackIdx = (upper - trackStart) / trackStep;
	}

    return Range<int>(startTrackIdx, endTrackIdx);
}

Range<int> SPRoute::GetTrackRange(int layerIdx, Rect2D<int> rect, bool expand)
{
    int lower, upper, startTrackIdx, endTrackIdx;
    int trackStart, trackStep;
    string direction = lefDB.layers.at(layerIdx).direction;
    string layerName = lefDB.layers.at(layerIdx).name;

    if(defDB.layeridx2trackidx.count(layerIdx) == 0)
    {
        cout << "unable to find layer in layer tracks: " << layerName << endl;
        exit(1);
    }

    if(direction == "HORIZONTAL") // for pessimism, disable one more track 
    {
        lower = rect.lowerLeft.y;
        upper = rect.upperRight.y;
    }
    else if(direction == "VERTICAL")
    {
        lower = rect.lowerLeft.x;
        upper = rect.upperRight.x;
    }
    else
    {
        cout << "unkown direction of layer : " << layerIdx << endl;
        exit(1);
    }

    int trackIdx = defDB.layeridx2trackidx.find(layerIdx)->second;
    trackStart = defDB.tracks.at(trackIdx).start;
    trackStep = defDB.tracks.at(trackIdx).step;

    if(expand)
    {
	    startTrackIdx = (lower - trackStep - trackStart) / trackStep;
	    startTrackIdx = (startTrackIdx < 0)? 0 : startTrackIdx;
	    endTrackIdx = (upper + trackStep - trackStart) / trackStep;
	}
	else
	{
		startTrackIdx = (lower - trackStart) / trackStep + 1;
	    endTrackIdx = (upper - trackStart) / trackStep;
	}

    return Range<int>(startTrackIdx, endTrackIdx);
}



void SPRoute::WriteRoutedGrid(FILE* fp, std::set<sproute::Point3D<int>>& routedGrid)
{
	for(auto p : routedGrid)
	{
		int llx  = defDB.xGcellBoundaries.at(p.x);		
		int lly = defDB.yGcellBoundaries.at(p.y);
		int urx = defDB.xGcellBoundaries.at(p.x + 1);
		int ury  = defDB.yGcellBoundaries.at(p.y + 1);
		int layer = p.z;

		if(layer * 2 < lefDB.layers.size())
		{
			string layerName = lefDB.layers.at(layer * 2).name;
			fprintf(fp, "%d %d %d %d %s\n", llx, lly, urx, ury, layerName.c_str());
		}
		else {
			cout << "Error in writing guide: exceeds top layer! " << endl;
			exit(1);
		}
	}
}

void SPRoute::WriteRoute3D(char routingfile3D[])
{
	short *gridsX, *gridsY, *gridsL;
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	using Point3D = sproute::Point3D<int>;
	
	fp=fopen(routingfile3D, "w");
    if (fp == NULL) {
        printf("Error in opening %s\n", routingfile3D);
        exit(1);
    }

	for(netID=0;netID<numValidNets;netID++)
	{
		string netName(nets[netID]->name);
		bool print = false;
		fprintf(fp, "%s\n", nets[netID]->name);
		fprintf(fp, "(\n");
        treeedges=sttrees[netID].edges;
		deg=sttrees[netID].deg;
		
		nodes = sttrees[netID].nodes;

		int grNetID = defDB.netName2netidx.find(nets[netID]->name)->second; 
		auto grNet = grGen.grnets.at(grNetID);

		std::set<Point3D> routedGrid;

		for(auto p : grNet.pinRegion)
		{
			int botL = p.z - 1;
			int topL = botL + 2;
            if(netName == "Reset") {
                cout << p.x << " " << p.y << " " << p.z << endl;
            }
			for (int layer = botL; layer <= topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				assert(p.x < xGrid && p.y < yGrid);
				if(layer < numLayers)
                	routedGrid.insert(Point3D(p.x, p.y, layer));
            }
		}

		for(int i = 0; i < deg; i++)
		{	
			if(nodes[i].botL == 0)
				nodes[i].topL = (nodes[i].topL >= 2)? nodes[i].topL : 2;

			for (int layer = nodes[i].botL; layer <= nodes[i].topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				assert(nodes[i].x < xGrid && nodes[i].y < yGrid);
				if(layer < numLayers)
					routedGrid.insert(Point3D(nodes[i].x, nodes[i].y, layer));
			}
		}

		for(edgeID = 0 ; edgeID < 2*deg-3; edgeID++)
		{
			edge = sttrees[netID].edges[edgeID];
			treeedge = &(treeedges[edgeID]);
			if (treeedge->len > 0) {
				
				routeLen = treeedge->route.routelen;
				gridsX = treeedge->route.gridsX;
				gridsY = treeedge->route.gridsY;
				gridsL = treeedge->route.gridsL;

				if(print)
				{
					cout << "routelen: " << routeLen << endl;
				}

				string direction;
				bool output = false;
				int start, end;
				for(int output_cnt = 0; output_cnt < 1; output_cnt++)
				{
					for (i = 0; i <= routeLen; i ++) {
						routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i]));
						if(i % 5 == 3) {
							if(gridsL[i] <= numLayers - 2) {
								routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i] + 1));
								//cout << "add top" << endl;
							}
							else if(gridsL[i] >= 2) {//metal5 adding a metal4 
								routedGrid.insert(Point3D(gridsX[i], gridsY[i], gridsL[i] - 1));
							}
						}
					}
				}
			}
		}

		writeRoutedGrid(fp, routedGrid);
		fprintf(fp, ")\n");
	}

	for(netID=0;netID<numInvalidNets;netID++)
	{
		fprintf(fp, "%s\n", invalid_nets[netID]->name);
		fprintf(fp, "(\n");

		string netName(invalid_nets[netID]->name);
		bool print = false;

		if(defDB.netName2netidx.count(netName) == 0)
		{
			cout << "Error: unable to find net: " << netName << endl;
			exit(1);
		}
		int grNetID = defDB.netName2netidx.find(netName)->second; 
		auto grNet = grGen.grnets.at(grNetID);

		std::set<Point3D> routedGrid;

		for(auto p : grNet.pinRegion)
		{
			int botL = p.z - 1;
			int topL = botL + 2;

			for (int layer = botL; layer <= topL; layer++) //this is not good, assuming all pins are on botL layer.
			{
				if(layer * 2 < lefDB.layers.size())
				{
					routedGrid.insert(Point3D(p.x, p.y, layer));
				}
			}
		}

		writeRoutedGrid(fp, routedGrid);
		fprintf(fp, ")\n");
	}

	fclose(fp);
}


void SPRoute::WriteOverflow(string overflowFile)
{
	short *gridsX, *gridsY, *gridsL;
	int netID, d, i,k, edgeID,nodeID,deg, lastX, lastY,lastL, xreal, yreal,l, routeLen;
	TreeEdge *treeedges, *treeedge;
	FILE *fp;
	TreeNode *nodes;
	TreeEdge edge;
	using Point3D = sproute::Point3D<int>;
	
	fp=fopen(overflowFile.c_str(), "w");
    if (fp == NULL) {
        printf("Error in opening %s\n", overflowFile.c_str());
        exit(1);
    }

	std::set<Point3D> overflowGrid;
	string overflow_name = "overflow";
	fprintf(fp, "%s\n", overflow_name.c_str());
	fprintf(fp, "(\n");
        
	for (int k = 0; k < numLayers; k++) {
		for (int i = 0; i < yGrid; i++) {
			for (int j = 0; j < xGrid - 1; j++) {
				int grid = i * (xGrid - 1) + j + k * (xGrid - 1) * yGrid;
				int overflow = h_edges3D[grid].usage - h_edges3D[grid].cap;

				if (overflow > 0) {
					overflowGrid.insert(Point3D(j, i, k));
				}
			}
		}
		for (int i = 0; i < yGrid - 1; i++) {
			for (int j = 0; j < xGrid; j++) {
				int grid = i * xGrid + j + k * xGrid * (yGrid - 1);
				
				int overflow = v_edges3D[grid].usage - v_edges3D[grid].cap;

				if (overflow > 0) {
					overflowGrid.insert(Point3D(j, i, k));
				}
			}
		}
	}

	writeRoutedGrid(fp, overflowGrid);
	fprintf(fp, ")\n");

	fclose(fp);
}

} //namespace sproute;
