#ifndef ALLINFOCLASS_H
#define ALLINFOCLASS_H
#include "ros/ros.h"
#include "an_dynamic_planner/mapCell.h"
#include "an_messages/lanes.h"
#include "an_messages/lane.h"
#include <vector>
#include <cmath>
#include <map>
using namespace std;
class allInfoClass{
public:
	//map cell's width is 0.74m, 5 tracks per lane
	//length is 1m
	double cell_w;
	double cell_l;
	double lane_w;
	double width, length;
	double outer_radius, inner_radius;
	ros::Time startTime;
 	std::vector<std::vector<mapCell> > myMap;
 	
 	//Edges of a lane
 	struct edge{
 		double left_edge;
 		double right_edge;
 	};
 	//Obstacles
 	struct obstacle{
		vector<vector<mapCell::carState> > path;
		vector<double> probability;
		obstacle(int path_num);
		obstacle();
	};
	map<double, vector<mapCell::carState> > allObsEveryTimeStep;
	vector<mapCell::carState> straight_MP;
	vector<mapCell::carState> right_MP;
	vector<mapCell::carState> left_MP;

	//};

 	//Map information
 	int lane_num, track_num, station_num;
 	mapCell::carState global_goal, global_start;
 	std::vector<edge> edges; //[lane]->{left, right}
 	std::vector<obstacle> obs;
 	double stCost, turnCost;
//allMPStruct allMP;
	

	

	//Function
	allInfoClass();
	//Init map
 	void initMap(an_messages::lanes msg, int station_num, int track_num, double cell_w, double cell_l);
 	//Get a pointer points to cell in {lane li, track ti, station si}
 	mapCell* getCell(int li, int ti, int si);
 	

};



#endif
