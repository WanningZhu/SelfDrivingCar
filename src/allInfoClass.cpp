#include "an_dynamic_planner/allInfoClass.h"
//#include "allInfoClass.h"
allInfoClass::allInfoClass(){ }
void allInfoClass::initMap(an_messages::lanes msg, int station_num, int track_num, double cell_w, double cell_l){
	//map size 
	lane_w = msg.lanes[0].width;
 	lane_num = msg.lanes.size();
 	this->station_num = station_num;
 	this->track_num = track_num;
 	this->cell_l = cell_l;
 	this->cell_w = cell_w;
 	myMap.resize(lane_num*track_num);
 	for(int l = 0; l < lane_num; l++){
 		//Lane edge of every lane
 		edge e;
 		e.left_edge = msg.lanes[l].leftedge[0].y;
 		e.right_edge = msg.lanes[l].rightedge[0].y;
 		edges.push_back(e);
 		ROS_DEBUG("[InitMap_Lane] %d lane [%0.2f, %0.2f]", l, e.left_edge, e.right_edge);
 
 		for(int t = 0; t < track_num; t++){
 			for(int s = 0; s < station_num; s++){
 				//cell center position
 				double x_c = cell_l*s;
 				double y_c = edges[0].left_edge + l*lane_w + (t+0.5)*cell_w;
 				mapCell::carState state(x_c, y_c, 0, 0);//car state is at cell's center
 				mapCell cell(state, l, t, s);
 				myMap[l*track_num+t].push_back(cell);
 			}
			ROS_DEBUG("[InitMMap] {l%d t%d}: start: x=%.2f, y=%.2f; end: x=%.2f, y=%.2f", l, t, myMap[l*track_num+t][0].x, myMap[l*track_num+t][0].y, myMap[l*track_num+t].back().x, myMap[l*track_num+t].back().y);
 		}
 	}
}

mapCell* allInfoClass::getCell(int li, int ti, int si){
	if(li >= lane_num || ti >= track_num || si >= station_num){
		ROS_ERROR("[GetCell](%d %d %d) Get failed! Out of bound!", li, ti, si);
		return NULL;
	}
	return &myMap[li*track_num + ti][si];
}


allInfoClass::obstacle::obstacle(int path_num){
	path.resize(path_num);
	probability.resize(path_num);
}
allInfoClass::obstacle::obstacle(){}