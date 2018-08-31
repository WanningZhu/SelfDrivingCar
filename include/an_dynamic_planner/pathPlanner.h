#ifndef PATHPLANNER_H
#define PATHPLANNER_H
#include "ros/ros.h"
#include <vector>
#include <cmath>
#include <limits>
#include <map>
#include "an_dynamic_planner/mapCell.h"
#include "an_dynamic_planner/allInfoClass.h"
#include "an_dynamic_planner/minHeap.h"

#include "geometry_msgs/Point.h"
#include "an_messages/lanes.h"
#include "an_messages/lane.h"
#include "an_messages/obstacles.h"
#include "an_messages/obstacle.h"
#include "an_messages/trajectory.h"
#include "an_messages/traj_pt.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
using namespace std;
class pathPlanner{
public:
	bool getData_;
	//Ponter pointer to allInfoClass, manipulate all data related to path planning,
	allInfoClass* data_ptr;
	//mapCell::carState* g_goal;
	//mapCell::carState* g_start;

	double lane_w;
	double cell_w;
	double cell_l;
	//Constructor
	pathPlanner();
	pathPlanner(allInfoClass* ptr);
	//Generate trajectory, external accessible
	an_messages::trajectory generateTraj(ros::Duration d);


	void test();
private:
	//bool arrived_;
	//Input position, return corresponding cell pointer. 
	//Point in right edge belonging to next lane;
	mapCell* positionToCell(double x, double y);
	void setH(mapCell* cell_ptr);
	vector<mapCell*> generateSuccessors(mapCell* cur_cell);
	mapCell* putIntoSuccessors(mapCell* cur_cell, vector<mapCell::carState>* MP, vector<mapCell*>* successors);
	bool checkHit(mapCell::carState* start, vector<mapCell::carState>* dir_MP);
	bool checkHitHelper(double c_x, double c_y, mapCell::carState* ob);
	
	bool ARASearch(mapCell* start_cell, mapCell* goal_cell, MinHeap* opened, vector<mapCell*>* incons,  double eps, int count);
	void combineOpenedAndIncons(MinHeap* opened, vector<mapCell*>* incons);
	double callEpsPrime(double eps, MinHeap* opened, mapCell* goal_cell);
	//Update f value of all member in opened
	void updateWithNewEps(MinHeap* opened, double new_eps);
	an_messages::trajectory backtrackTraj(mapCell* goal_cell, mapCell* start_cell, ros::Duration d);
	double MPCost(mapCell* cur, mapCell* next);
	bool isInVector(mapCell* cell, vector<mapCell*>* vec);
};


#endif