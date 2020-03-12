#ifndef GRGEN_H
#define GRGEN_H

#include "header.h"
#include "parserDataType.h"
#include "defDataBase.h"

namespace parser
{

int find_Gcell(int pin_in, std::vector<int> GcellBoundaries)
{
	auto it = std::upper_bound(GcellBoundaries.begin(), GcellBoundaries.end(), pin_in);
	int x = std::distance(GcellBoundaries.begin(), it) - 1;

	//assert(x >= 0);
	//assert(it != GcellBoundaries.end() || pin_in == *(GcellBoundaries.begin() + GcellBoundaries.size() - 1));
	if(x == -1)
		x++;
	else if(it == GcellBoundaries.end() && pin_in != *(GcellBoundaries.begin() + GcellBoundaries.size() - 1))
		x--;
	/*if(!(x >= 0 && it != GcellBoundaries.end()))
	{
		cout << " tmp break " << endl;
	}*/
	return x;
}

class CongestionMap {
	float* hCongestion;
	float* vCongestion;
	int xGrid;
	int yGrid;
	int numLayers;
public:
	CongestionMap(int numLayers, int xGrid, int yGrid) {
		this->xGrid = xGrid;
		this->yGrid = yGrid;
		this->numLayers = numLayers;
		hCongestion = new float [numLayers * (xGrid - 1) * yGrid];
		vCongestion = new float [numLayers * xGrid * (yGrid - 1)];

	};

	float getCongestion(int layer, int x, int y, bool horizontal) {
		if(horizontal) {
			int grid = y*(xGrid-1) + x + layer*(xGrid-1)*yGrid;
			return hCongestion[grid];
		}
		else {
			int grid = y*xGrid + x + layer*xGrid*(yGrid - 1);
			return vCongestion[grid];
		}
	}

	void setCongestion(int layer, int x, int y, bool horizontal, float cong) {
		if(horizontal) {
			int grid = y*(xGrid-1) + x + layer*(xGrid-1)*yGrid;
			//cout << x << " " << y << " " << layer << " " << cong << endl;
			hCongestion[grid] = cong;
		}
		else {
			int grid = y*xGrid + x + layer*xGrid*(yGrid - 1);
			vCongestion[grid] = cong;
		}
	}
};

class grNet
{
public:
	string name;
	int idx;
	int numPins;
	int minWidth;
	std::vector<Point3D<int>> pins;
	std::set<Point3D<int>> pinRegion;
	grNet() {
		minWidth = 1;
	}

	void clear()
	{
		name = "";
		idx = -1;
		numPins = 0;
		minWidth = 1;
	}

};



class grGenerator
{
public:
	Point3D<int> grid;
	std::vector<int> vCap; 
	std::vector<int> hCap;

	Point2D<int> gcellBegin;
	Point2D<int> gcellSize;

	int numNets;
	std::vector<grNet> grnets;
	std::map<string, int> netName2grnetidx;

	int minWidth;
	int minSpacing;
	int viaSpacing;

	int numAdjust;
	galois::InsertBag<CapReduction>* capReductions_p;

	grGenerator(lefDataBase& lefDB, defDataBase& defDB, galois::InsertBag<CapReduction>& capReductions)
	{
		capReductions_p = &capReductions;
		minWidth = 1;
		minSpacing = 1;
		viaSpacing = 1;
		gcellBegin.set(defDB.xGcellBoundaries.front(), defDB.yGcellBoundaries.front());
		numAdjust = 0;

		int numLayers = 0;
		for(auto layer : lefDB.layers)
		{
			if(layer.type == "ROUTING")
				numLayers++;
		}

		grid.x = 0;
		grid.y = 0;
		grid.z = numLayers;
		int maxGcellCnt_x = 0, maxGcellCnt_y = 0;
		for(auto gcellGrid : defDB.gcellGrids)
	    {
	        if(gcellGrid.direction == "X")
	        {
	            grid.x += (gcellGrid.numBoundaries - 1);
	            if(gcellGrid.numBoundaries > maxGcellCnt_x) {
	            	maxGcellCnt_x = gcellGrid.numBoundaries;
	            	gcellSize.x = gcellGrid.step;
	            }
	        }
	        else if(gcellGrid.direction == "Y")
	        {
	        	grid.y += (gcellGrid.numBoundaries - 1);
	        	if(gcellGrid.numBoundaries > maxGcellCnt_y) {
	            	maxGcellCnt_y = gcellGrid.numBoundaries;
	            	gcellSize.y = gcellGrid.step;
	            }
	        }
	        else
	        {
	            cout << "unknown gcell direction: " << gcellGrid.direction << endl;
	            exit(1);
	        }
	    }
	    cout << grid.x << " " << grid.y << " " << grid.z << endl;

	    cout << "gcellbegin: " << gcellBegin.x << " " << gcellBegin.y <<endl;
	    cout << "gcellSize: " << gcellSize.x << " " << gcellSize.y <<endl;


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
								if(hCap.size() != 0)
								{
									hCap.push_back(gcellSize.y / track.step);
									vCap.push_back(0);
								}
								else
								{
									hCap.push_back(0);
									vCap.push_back(0);
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
								if(vCap.size() != 0)
								{
									vCap.push_back(gcellSize.x / track.step);
									hCap.push_back(0);
								}
								else
								{
									vCap.push_back(0);
									hCap.push_back(0);
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
		for(auto cap : vCap)
			cout << cap << " ";
		cout << endl;

		cout << " hcap: " ;
		for(auto cap : hCap)
			cout << cap << " ";
		cout << endl;



		int cnt = 0;
		numNets = defDB.nets.size();
		for(auto net : defDB.nets)
		{
			grNet tmpGRnet;

			tmpGRnet.name = net.name;
			tmpGRnet.idx = cnt;
			cnt++;

			int numPins = net.pinNames.size();
			if(numPins > 1000)
				cout << "large net: " << net.name << endl;
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
					z = lefDB.layer2idx.find(iopin.layerName)->second / 2 + 1;
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
								int pinRegionz = lefDB.layer2idx.find(layerRects.layerName)->second / 2 + 1;
								int trackIdx = defDB.layerName2trackidx.find(layerRects.layerName)->second;
								int trackStep = defDB.tracks.at(trackIdx).step;
								for(auto rect: layerRects.rects)
								{
									int xmin = find_Gcell(rect.lowerLeft.x - trackStep, defDB.xGcellBoundaries);
            						int ymin = find_Gcell(rect.lowerLeft.y - trackStep, defDB.yGcellBoundaries);
            						int xmax = find_Gcell(rect.upperRight.x + trackStep, defDB.xGcellBoundaries);
            						int ymax = find_Gcell(rect.upperRight.y + trackStep, defDB.yGcellBoundaries);

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
			grnets.push_back(tmpGRnet);
			netName2grnetidx.insert( pair< string, int> (tmpGRnet.name, tmpGRnet.idx));
			/*for(auto pinRegion : tmpGRnet.pinRegion)
			{
				pinRegion.print();
			}*/
			tmpGRnet.pinRegion.clear();
			tmpGRnet.clear();

		}


	}

	void write(string fileName)
	{
		ofstream outfile(fileName);
		outfile << "grid " << grid.x << " " << grid.y << " " << grid.z << endl;

		outfile << "vertical capacity ";
		for(auto cap : vCap)
			outfile << 2*cap << " ";
		outfile << endl;

		outfile << "horizontal capacity ";
		for(auto cap : hCap)
			outfile << 2*cap << " ";
		outfile << endl;

		outfile << "minimum width ";
		for(int i = 0; i < grid.z; i++)
			outfile << minWidth << " ";
		outfile << endl;

		outfile << "minimum spacing ";
		for(int i = 0; i < grid.z; i++)
			outfile << minSpacing << " ";
		outfile << endl;

		outfile << "via spacing ";
		for(int i = 0; i < grid.z; i++)
			outfile << viaSpacing << " ";
		outfile << endl;

		outfile << gcellBegin.x << " " <<gcellBegin.y << " " << gcellSize.x << " " << gcellSize.y << endl;

		outfile << endl;

		outfile << "num net " << numNets << endl;

		for(auto grnet : grnets)
		{
			outfile << grnet.name << " " << grnet.idx << " " << grnet.numPins << " " << grnet.minWidth << endl;
			for(auto pin : grnet.pins)
			{
				outfile << pin.x << " " << pin.y << " " << pin.z << endl;
			}
		}
		outfile << endl;
		outfile << endl;

		outfile << "cap reduction "  << endl;

		for(auto capReduction : *(this->capReductions_p))
			outfile << capReduction.x << " " << capReduction.y << " " << capReduction.z << " " << capReduction.newCap << endl;
	}
};

}

#endif
