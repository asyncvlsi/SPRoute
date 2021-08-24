#include "sproute.h"

namespace sproute {

int SPRoute::InvalidNetsAdjLayer(int x, int y, int l, int xGrid, int yGrid) {

    int adj = 0;
    if (lefDB.layers.at(l * 2).direction == "HORIZONTAL")//horizontal 
    {
        
        if(x < xGrid - 1) { //not the right boundary 
            int grid3D = y*(xGrid-1)+x+l*(xGrid-1)*yGrid;
            if(h_edges3D[grid3D].cap > 0) {
                h_edges3D[grid3D].cap -= 1;
                h_edges3D[grid3D].red += 1;
                int grid2D =y*(xGrid-1)+x;
                h_edges[grid2D].cap -=1;
                h_edges[grid2D].red += 1;
                adj++;
            }
        }
        
        if(x > 0) { //not the left boundary
            x -= 1;
            int grid3D = y*(xGrid-1)+x+l*(xGrid-1)*yGrid;
            if(h_edges3D[grid3D].cap > 0) {
                h_edges3D[grid3D].cap -= 1;
                h_edges3D[grid3D].red += 1;
                int grid2D =y*(xGrid-1)+x;
                h_edges[grid2D].cap -= 1;
                h_edges[grid2D].red += 1;
                adj++;
            }
        }
    
    } 
    else if (lefDB.layers.at(l * 2).direction == "VERTICAL")//vertical 
    {
        if(y < yGrid - 1) { // not the top boundary
            int grid3D = y*xGrid + x + l*xGrid*(yGrid-1);
            if(v_edges3D[grid3D].cap > 0) {
                v_edges3D[grid3D].cap -= 1;
                v_edges3D[grid3D].red += 1;
                int grid2D = y * xGrid + x;
                v_edges[grid2D].cap -= 1;
                v_edges[grid2D].red += 1;
                adj++;
            }
        }

        if(y > 0) { // not the bot boundary
            y -= 1;
            int grid3D = y*xGrid + x + l*xGrid*(yGrid-1);
            if(v_edges3D[grid3D].cap > 0) {
                v_edges3D[grid3D].cap -= 1;
                v_edges3D[grid3D].red += 1;
                int grid2D = y * xGrid + x;
                v_edges[grid2D].cap -= 1;
                v_edges[grid2D].red += 1;
                adj++;
            }
        }
    }
    else
    {
        cout << "unknown layer direction in invalidNetsAdj" << endl;
        exit(1);
    }
    return adj;

}

void SPRoute::InvalidNetsAdj(Net** invalid_nets, int xGrid, int yGrid) {

    int num_adj = 0;
    for(int i = 0 ; i < numInvalidNets; i++) {

        Net* net= invalid_nets[i];
        int x = net->pinX[0];
        int y = net->pinY[0];
        int l = net->pinL[0];  //starting from 1

        if(net->numPins > 1 && l != 1) {
            std::cout << "An invalid nets is not on bottom layer? " << std::endl;
            std::cout << net->name << " " << net->numPins << endl;
            std::cout << x << " " << y << " " << l << endl;
            exit(1);
        }

        num_adj += InvalidNetsAdjLayer(x, y, 1, xGrid, yGrid);
        num_adj += InvalidNetsAdjLayer(x, y, 2, xGrid, yGrid);
    }
    if(verbose_ > none)
        cout << "num of invalid net : " << numInvalidNets << "adj: " << num_adj << endl;

}

}
