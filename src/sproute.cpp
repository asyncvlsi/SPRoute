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
#include "DataProc.h"
#include "RSMT.h"
#include "ripup.h"
#include "utility.h"
#include "route.h"
//#include "maze3D.h"
//#include "maze_finegrain.h"
//#include "maze_finegrain_lateupdate.h"
//#include "maze_lock.h"

#include "algo.h"
#include "grGen.h"
//#include "detpart.h"
//#include "a_star.h"
//#include "detpart_astar.h"
//#include "detpart_astar_data.h"
//#include "detpart_astar_local.h"

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
        sproute_db::Macro tmpMacro;
        int idx = lefDB.macros.size();
        tmpMacro.name = phydb_macro.GetName();
        tmpMacro.origin.x = phydb_macro.GetOriginX();
        tmpMacro.origin.y = phydb_macro.GetOriginY();
        tmpMacro.size.x = phydb_macro.GetWidth();
        tmpMacro.size.y = phydb_macro.GetHeight();

        auto phydb_pins = phydb_macro.GetPinsRef();
        for(auto phydb_pin : phydb_pins) {
            sproute_db::Pin tmpPin;
            tmpPin.name = phydb_pin.GetName();
            tmpPin.direction = phydb::SignalDirectionStr(phydb_pin.GetDirection());
            tmpPin.use = phydb::SignalUseStr(phydb_pin.GetUse());
            
            auto phydb_layer_rects = phydb_pin.GetLayerRectRef();
            for(auto phydb_layer_rect : phydb_layer_rects) {
                sproute_db::LayerRect tmpLayerRect;
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
            sproute_db::LayerRect tmpLayerRect;
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
        sproute_db::lefVia tmpVia;
        int idx = lefDB.vias.size();
        tmpVia.name = phydb_via.GetName();

        auto phydb_layer_rects = phydb_via.GetLayerRectsRef();
        for(auto phydb_layer_rect : phydb_layer_rects) {
            sproute_db::LayerRect tmpLayerRect;
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
        sproute_db::Layer tmpLayer;
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
                sproute_db::Spacing tmpSpacing;
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
        sproute_db::Component tmpComponent;
        int idx = defDB.components.size();

        tmpComponent.idx = idx;
        tmpComponent.name = phydb_component.GetName();
        tmpComponent.macroName = phydb_component.GetMacro()->GetName();
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
        sproute_db::Track tmpTrack;

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
        sproute_db::IOPin tmpIOPin;

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
        sproute_db::SNet tmpSNet;

        tmpSNet.name = phydb_snet.GetName();
        auto phydb_paths = phydb_snet.GetPathRef();
        for(auto phydb_path : phydb_paths) {
            sproute_db::Path tmpPath;
            
            tmpPath.layerName = phydb_path.GetLayerName();
            tmpPath.width = phydb_path.GetWidth();
            tmpPath.shape = phydb_path.GetShape();
            tmpPath.viaName = phydb_path.GetViaName();
            tmpPath.beginExt = phydb_path.GetBeginExt();
            tmpPath.endExt = phydb_path.GetEndExt();
            tmpPath.rect.set(phydb_path.GetRect().LLX(), phydb_path.GetRect().LLX(), phydb_path.GetRect().LLX(), phydb_path.GetRect().LLX());
            tmpPath.begin.x = phydb_path.GetBegin().x;
            tmpPath.begin.y = phydb_path.GetBegin().y;
            tmpPath.end.x = phydb_path.GetEnd().x;
            tmpPath.end.y = phydb_path.GetEnd().y;

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
        sproute_db::Net tmpNet;

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
        sproute_db::GcellGrid tmpGcellGrid;

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
    sproute_db::GcellGrid tmpGcellGrid_x, tmpGcellGrid_x1, tmpGcellGrid_y, tmpGcellGrid_y1;
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
    sproute_db::Point2D<int> maxBoundaryCnt;
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
                        sproute_db::Rect2D<float> tmpRect;
                        tmpRect.set(path.rect.lowerLeft.x, path.rect.lowerLeft.y, path.rect.upperRight.x, path.rect.upperRight.y);
                        defDB.origOBS.at(layerIdx).push_back(tmpRect);
                    }
                    else
                    {
                        sproute_db::Rect2D<float> tmpRect;
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
                    sproute_db::lefVia via = lefDB.vias.at(viaIdx);
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
                                sproute_db::Rect2D<float> tmpRect;
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
                    sproute_db::defVia via = defDB.vias.at(viaIdx);
                    int xshift = ((float)via.numCutCols - 1 ) / 2 * via.cutSpacing.x + via.botEnc.x;
                    int yshift = ((float)via.numCutRows - 1 ) / 2 * via.cutSpacing.y + via.botEnc.y;

                    string botLayer = via.layers[0];
                    string topLayer = via.layers[2];

                    int botLayerIdx = lefDB.layer2idx.find(botLayer)->second;
                    int topLayerIdx = lefDB.layer2idx.find(topLayer)->second;

                    sproute_db::Rect2D<float> tmpRect;
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
        sproute_db::Layer& layer = lefDB.layers.at(i);

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
                sproute_db::Rect2D<int> expandRect;
                expandRect.lowerLeft.x = rect.lowerLeft.x - expand;
                expandRect.lowerLeft.y = rect.lowerLeft.y - expand;
                expandRect.upperRight.x = rect.upperRight.x + expand;
                expandRect.upperRight.y = rect.upperRight.y + expand;
                defDB.designRuleOBS.at(i).push_back(expandRect);
            }
            else { //standard width wire
                sproute_db::Rect2D<int> intRect;
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
                    sproute_db::Gcell& gcell = defDB.getGcell(x, y, i/2);
                        
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

                    sproute_db::Gcell& nextGcell = defDB.getGcell(next_x, next_y, i/2);

                    //if(debug)
                    //    cout << x << " " << y << endl;

                    if(gcell.obs_state == sproute_db::NO_OBS && nextGcell.obs_state == sproute_db::NO_OBS)
                        continue;

                    int numUsedTrack = 0;

                    if(gcell.obs_state == sproute_db::NO_OBS) { // if current gcell is empty, only consider next gcell
                        for(int trackIdx = 0; trackIdx < nextGcell.numTracks; trackIdx++) {
                            if(nextGcell.trackUsed[trackIdx])
                                numUsedTrack++;
                        }
                    }
                    else if(nextGcell.obs_state == sproute_db::NO_OBS) { // if next gcell is empty, only consider current gcell
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

                    sproute_db::CapReduction tmpCapReduction;
                    tmpCapReduction.x = x;
                    tmpCapReduction.y = y;
                    tmpCapReduction.z = i / 2;
                    if(gcell.obs_state == sproute_db::NO_OBS)
                        tmpCapReduction.newCap = nextGcell.numTracks - numUsedTrack;
                    else
                        tmpCapReduction.newCap = gcell.numTracks - numUsedTrack;
                    if(tmpCapReduction.newCap < 0) {
                        cout << " cap adjust < 0, error! " << gcell.numTracks << " " << numUsedTrack << " x : " << x << " y: " << y << " z: " << i / 2 <<  endl;
                        cout << defDB.xGcellBoundaries.size() - 2 << " " << defDB.yGcellBoundaries.size() - 2 << endl;
                        exit(1);
                    }
                    //assert(tmpCapReduction.newCap >= 0);

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

            xGcell_start = sproute_db::find_Gcell(rect.lowerLeft.x - trackStep, defDB.xGcellBoundaries);
            yGcell_start = sproute_db::find_Gcell(rect.lowerLeft.y - trackStep, defDB.yGcellBoundaries);
            xGcell_end = sproute_db::find_Gcell(rect.upperRight.x + trackStep, defDB.xGcellBoundaries);
            yGcell_end = sproute_db::find_Gcell(rect.upperRight.y + trackStep, defDB.yGcellBoundaries);

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

void SPRoute::GcellInsertOBS(int x, int y, int z, sproute_db::Rect2D<int> rect, int OBSIdx)
{
    bool debug = false;
    sproute_db::Gcell& gcell = defDB.getGcell(x, y, z);
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
    sproute_db::Track track = defDB.tracks.at(trackIdx);
    int trackStart = track.start;
    int trackStep = track.step;


    int gcellTrackStart = (gcellLower - trackStart) / trackStep + 1; // both are inclusive, i.e. local 0-14
    int gcellTrackEnd = (gcellUpper - trackStart) / trackStep;

    sproute_db::Range<int> gcellRange(gcellTrackStart, gcellTrackEnd);


    if(gcell.numTracks == 0)
    {
        gcell.numTracks = gcellTrackEnd - gcellTrackStart + 1;
        if(gcell.numTracks > 15) {
            cout << "numtracks more than 15? " << gcell.numTracks << endl;
            exit(1);
        }
        for(int i = 0; i < gcell.numTracks; i++)
        {
            gcell.trackUsed[i] = false;
        }
    }

    int std_width = lefDB.layers.at(z*2).width * defDB.dbuPerMicro;
    int rect_width = (direction == "HORIZONTAL")? rect.upperRight.y - rect.lowerLeft.y : rect.upperRight.x - rect.lowerLeft.x;

    bool expand = (rect_width == std_width)? false : true;

    sproute_db::Range<int> OBSTrackRange = GetTrackRange(z*2, rect, expand);
    sproute_db::Range<int> overlapRange = RangeIntersection(gcellRange, OBSTrackRange);
    if(overlapRange.start == 0 && overlapRange.end == 0)
        return;
    int localStart = overlapRange.start - gcellTrackStart;
    int localEnd = overlapRange.end - gcellTrackStart;


    for(int i = localStart; i <= localEnd; i++)
    {
        gcell.trackUsed[i] = true;
    }

    gcell.OBSIDlist.push_back(OBSIdx);

    sproute_db::Point2D<int> GcellLowerLeft(defDB.xGcellBoundaries.at(x), defDB.yGcellBoundaries.at(y));
    sproute_db::Point2D<int> GcellUpperRight(defDB.xGcellBoundaries.at(x + 1), defDB.yGcellBoundaries.at(y + 1));

    if(rect.lowerLeft.x <= GcellLowerLeft.x && rect.lowerLeft.y <= GcellLowerLeft.y && rect.upperRight.x >= GcellUpperRight.x && rect.upperRight.y >= GcellUpperRight.y)
        gcell.obs_state = sproute_db::FULLYCOVERED;
    else
    {
        gcell.obs_state = sproute_db::HAS_OBS;
    }

}

sproute_db::Range<int> SPRoute::RangeIntersection(sproute_db::Range<int> r1, sproute_db::Range<int> r2)
{
    sproute_db::Range<int> tmp;
    tmp.start = (r1.start > r2.start)? r1.start : r2.start;
    tmp.end = (r1.end < r2.end)? r1.end : r2.end;
    if(tmp.start > tmp.end)
        return sproute_db::Range<int>(0, 0);
    else
        return tmp;
} 

sproute_db::Range<float> SPRoute::RangeIntersection(sproute_db::Range<float> r1, sproute_db::Range<float> r2)
{
    sproute_db::Range<float> tmp;
    tmp.start = (r1.start > r2.start)? r1.start : r2.start;
    tmp.end = (r1.end < r2.end)? r1.end : r2.end;
    if(tmp.start > tmp.end)
        return sproute_db::Range<float>(0, 0);
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
        grGen.vCap.reserve(numLayers);
        grGen.hCap.reserve(numLayers);

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
									grGen.hCap.push_back(0); //metal1 cap = 0
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
									grGen.vCap.push_back(0); //metal1 cap = 0
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
        cout << "total number of nets: " << grGen.numNets << endl;
        grGen.grnets.reserve(grGen.numNets);
        grGen.grnets.resize(grGen.numNets);

		//for(auto net : defDB.nets)   //TODO: This should be parallelized
		//{
 
			cout << "enters here? " << endl;
     galois::do_all(
		 	galois::iterate( 0, grGen.numNets),
				[&](int idx) {
			sproute_db::grNet& tmpGRnet = grGen.grnets[idx];
			sproute_db::Net& net = defDB.nets[idx];
            
			tmpGRnet.name = net.name;
			tmpGRnet.idx = cnt;
			cnt++;
			int numPins = net.pinNames.size();
			if(numPins > 1000)
				cout << "large net: " << net.name << " " << numPins << endl;
			tmpGRnet.numPins = numPins;
            tmpGRnet.pins.reserve(numPins);
			for(int i = 0; i < numPins; i++)
			{
				int x, y, z;
				string compName = net.componentNames.at(i);
				string pinName = net.pinNames.at(i);

				if(compName == "PIN") // pin is an IO pin
				{
					int IOPinIdx = defDB.iopin2idx.find(pinName)->second;
					sproute_db::IOPin iopin = defDB.iopins.at(IOPinIdx);

					x = iopin.location.x;
					y = iopin.location.y;
					int pinx = sproute_db::find_Gcell(x, defDB.xGcellBoundaries);
					int piny = sproute_db::find_Gcell(y, defDB.yGcellBoundaries);
					z = lefDB.layer2idx[iopin.layerName] / 2 + 1;
			        tmpGRnet.pinRegion.insert(sproute_db::Point3D<int>(pinx, piny, z));
                }
				else
				{
					int compIdx = defDB.component2idx.find(compName)->second;
					sproute_db::Component& component = defDB.components.at(compIdx);
					bool found = false;
					for(sproute_db::Pin& pin : component.macro.pins)
					{
						if(pinName == pin.name)
						{
							x = pin.layerRects.at(0).rects.at(0).lowerLeft.x;
							y = pin.layerRects.at(0).rects.at(0).lowerLeft.y;
							string layerName = pin.layerRects.at(0).layerName;
							z = lefDB.layer2idx.find(layerName)->second / 2 + 1;
							found = true;
							for(sproute_db::LayerRect& layerRect : pin.layerRects)
							{
							    int layerIdx = lefDB.layer2idx[layerRect.layerName];	
                                if(lefDB.layers[layerIdx].type != "ROUTING")
                                    continue;
                                int pinRegionz = layerIdx / 2 + 1;
                                int trackIdx = defDB.layeridx2trackidx[layerIdx];
								int trackStep = defDB.tracks.at(trackIdx).step;
								for(sproute_db::Rect2D<float>& rect: layerRect.rects)
								{
									int xmin = sproute_db::find_Gcell(rect.lowerLeft.x - trackStep, defDB.xGcellBoundaries);
            						int ymin = sproute_db::find_Gcell(rect.lowerLeft.y - trackStep, defDB.yGcellBoundaries);
            						int xmax = sproute_db::find_Gcell(rect.upperRight.x + trackStep, defDB.xGcellBoundaries);
            						int ymax = sproute_db::find_Gcell(rect.upperRight.y + trackStep, defDB.yGcellBoundaries);

                                    xmin = (xmin < 0)? 0 : xmin;
                                    ymin = (ymin < 0)? 0 : ymin;
                                    xmax = (xmax >= grGen.grid.x)? grGen.grid.x - 1: xmax;
                                    ymax = (ymax >= grGen.grid.y)? grGen.grid.y - 1: ymax;
            						//if(xmin != xmax || ymin != ymax)
            						//	cout << "Pin covers two gcells: " << net.name << " " << component.name << "/" << pin.name << endl;
                                   
									for(int pinRegionx = xmin; pinRegionx <= xmax; pinRegionx++)
									{
										for(int pinRegiony = ymin; pinRegiony <= ymax; pinRegiony++)
											tmpGRnet.pinRegion.insert(sproute_db::Point3D<int>(pinRegionx, pinRegiony, pinRegionz));
									}
								}
							}
							break;
						}
					}
					assert(found == true);
				}
				sproute_db::Point3D<int> grpin(x, y, z);
				tmpGRnet.pins.push_back(grpin);
				//cout << x << " " << y << " " << z << endl;
			}
            /*cout << " 9 " << idx << endl;
			grGen.grnets[idx] = tmpGRnet;
            cout << " 10 " << endl;*/

		}, galois::steal());

		std::cout << "numnets: " << grGen.numNets << endl;

}



void SPRoute::ComputePinLocation(sproute_db::Component& component)
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


void SPRoute::ComputeOBSLocation(sproute_db::Component& component)
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

sproute_db::Range<int> SPRoute::GetTrackRange(int layerIdx, sproute_db::Rect2D<float> rect, bool expand)
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

    return sproute_db::Range<int>(startTrackIdx, endTrackIdx);
}

sproute_db::Range<int> SPRoute::GetTrackRange(int layerIdx, sproute_db::Rect2D<int> rect, bool expand)
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

    return sproute_db::Range<int>(startTrackIdx, endTrackIdx);
}



} //namespace sproute

