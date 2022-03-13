#include "sproute.h"
#include "soft_cap.h"
namespace sproute {

void SPRoute::MemPinAdj(int xGrid, int yGrid) {
    bool debug = false;

    for(auto& comp : defDB.components) {
        if(comp.locationType != "FIXED")
            continue;

        auto& macro = comp.macro;
        float xmin = -1;
        float xmax = -1;
        float ymin = -1;
        float ymax = -1;
        int xmin_cnt = 0, xmax_cnt = 0, ymin_cnt = 0, ymax_cnt = 0;

        for(auto& pin : macro.pins) {
            if(pin.use == "POWER" || pin.use == "GROUND") 
                continue;
            auto first_rect = pin.layerRects[0].rects[0];
            if(xmin == -1) {
                xmin = first_rect.lowerLeft.x;
                xmax = first_rect.lowerLeft.x;
                ymin = first_rect.lowerLeft.y;
                ymax = first_rect.lowerLeft.y;
            }
            else {
                xmin = min(first_rect.lowerLeft.x, xmin);
                xmax = max(first_rect.lowerLeft.x, xmax);
                ymin = min(first_rect.lowerLeft.y, ymin);
                ymax = max(first_rect.lowerLeft.y, ymax);
            }
        }
        for(auto& pin : macro.pins) {
            if(pin.use == "POWER" || pin.use == "GROUND") 
                continue;
            auto first_rect = pin.layerRects[0].rects[0];
            if(first_rect.lowerLeft.x == xmin) xmin_cnt++;
            if(first_rect.lowerLeft.x == xmax) xmax_cnt++;
            if(first_rect.lowerLeft.y == ymin) ymin_cnt++;
            if(first_rect.lowerLeft.y == ymax) ymax_cnt++;
        }

        if(debug) {
            cout << "component name " << comp.name << endl;
            cout << xmin_cnt << " " << xmax_cnt << " " << ymin_cnt << " " << ymax_cnt << endl;
        }

        for(auto& pin : macro.pins) {
            if(pin.use == "POWER" || pin.use == "GROUND") 
                continue;

            for(auto layerRect : pin.layerRects) {
                auto first_rect = layerRect.rects[0];
                int layerIdx = lefDB.layer2idx[layerRect.layerName];	
                if(lefDB.layers[layerIdx].type != "ROUTING")
                    continue;
                int l = layerIdx / 2;

                if(first_rect.lowerLeft.x == xmin && lefDB.layers[layerIdx].direction == "HORIZONTAL" && xmin_cnt != 1) {
                    int x = sproute_db::find_Gcell(first_rect.lowerLeft.x, defDB.xGcellBoundaries);
                    int y = sproute_db::find_Gcell(first_rect.lowerLeft.y, defDB.yGcellBoundaries);
                    if(x > 0)
                        x -= 1;
     
                    int grid3D = y*(xGrid-1)+x+l*(xGrid-1)*yGrid;
                    if(h_edges3D[grid3D].cap < hCapacity3D[l]) {
                        h_edges3D[grid3D].cap += 1;
                        h_edges3D[grid3D].red -= 1;
                        int grid2D = y * (xGrid - 1) + x;
                        h_edges[grid2D].cap += 1;
                        h_edges[grid2D].red -= 1;
                        if(debug) {
                            cout << "min h adj: " << x << " " << y << " " << l << endl;
                        }
                    }
                }

                if(first_rect.lowerLeft.x == xmax && lefDB.layers[layerIdx].direction == "HORIZONTAL" && xmax_cnt != 1) {
                    int x = sproute_db::find_Gcell(first_rect.lowerLeft.x, defDB.xGcellBoundaries);
                    int y = sproute_db::find_Gcell(first_rect.lowerLeft.y, defDB.yGcellBoundaries);
                    
                    int grid3D = y*(xGrid-1)+x+l*(xGrid-1)*yGrid;
                    if(h_edges3D[grid3D].cap < hCapacity3D[l]) {
                        h_edges3D[grid3D].cap += 1;
                        h_edges3D[grid3D].red -= 1;
                        int grid2D = y * (xGrid - 1) + x;
                        h_edges[grid2D].cap += 1;
                        h_edges[grid2D].red -= 1;
                        if(debug) {
                            cout << "max h adj: " << x << " " << y << " " << l << endl;
                        }
                    }
                }

                if(first_rect.lowerLeft.y == ymin && lefDB.layers[layerIdx].direction == "VERTICAL" && ymin_cnt != 1) {
                    int x = sproute_db::find_Gcell(first_rect.lowerLeft.x, defDB.xGcellBoundaries);
                    int y = sproute_db::find_Gcell(first_rect.lowerLeft.y, defDB.yGcellBoundaries);
                    if(y > 0)
                        y -= 1;
                    
                    int grid3D = y*xGrid + x + l*xGrid*(yGrid-1);
                    if(v_edges3D[grid3D].cap < vCapacity3D[l]) {
                        v_edges3D[grid3D].cap += 1;
                        v_edges3D[grid3D].red -= 1;
                        int grid2D = y * xGrid + x;
                        v_edges[grid2D].cap += 1;
                        v_edges[grid2D].red -= 1;
                        if(debug) {
                            cout << "v adj: " << x << " " << y << " " << l << endl;
                        }
                    }
                }

                if(first_rect.lowerLeft.y == ymax && lefDB.layers[layerIdx].direction == "VERTICAL" && ymax_cnt != 1) {
                    int x = sproute_db::find_Gcell(first_rect.lowerLeft.x, defDB.xGcellBoundaries);
                    int y = sproute_db::find_Gcell(first_rect.lowerLeft.y, defDB.yGcellBoundaries);
                    
                    int grid3D = y*xGrid + x + l*xGrid*(yGrid-1);
                    if(v_edges3D[grid3D].cap < vCapacity3D[l]) {
                        v_edges3D[grid3D].cap += 1;
                        v_edges3D[grid3D].red -= 1;
                        int grid2D = y * xGrid + x;
                        v_edges[grid2D].cap += 1;
                        v_edges[grid2D].red -= 1;
                        if(debug) {
                            cout << "v adj: " << x << " " << y << " " << l << endl;
                        }
                    }
                }

            }   // for layer
        } //for pin
    } //for component
}

}