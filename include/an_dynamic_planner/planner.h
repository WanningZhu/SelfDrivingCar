#ifndef PLANNER_H
#define PLANNER_H
#include <stdio.h>
#include <math.h>
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "geometry_msgs/Point.h"
#include "an_messages/lanes.h"
#include "an_messages/lane.h"
#include "an_messages/obstacles.h"
#include "an_messages/obstacle.h"
#include "an_messages/trajectory.h"
#include "an_messages/traj_pt.h"
#include "an_dynamic_planner/mapCell.h"
#include "an_dynamic_planner/allInfoClass.h"
#include "an_dynamic_planner/pathPlanner.h"
using namespace std;

class Planner_class{
public: 
	//Set all flags to false in constructor.
	Planner_class();
	//Initilization all variables and take in Motion primitives from rosparam
	bool Init();
	void Loop();
	ros::NodeHandle nh;
private:

	//Map information
	allInfoClass myData;

	//All flags
	bool new_plan_;
	bool pose_received_;
	bool goal_received_;
	bool obstacle_received_;
	bool map_received_;

	//Load Motion Primitive
	bool loadMP();
	//Call back functions
	void pose_sub_callback(const geometry_msgs::PoseStamped& msg);
	void goal_sub_callback(const geometry_msgs::PoseStamped& msg);
	void obstacle_sub_callback(const an_messages::obstacles& msg);
	void map_sub_callback(const an_messages::lanes& msg);
	void traj_sub_callback(const an_messages::trajectory& msg);
	void divideObsByTime(mapCell::carState obState);
	//Loop 
	void plan();
	
	
	



};

#endif