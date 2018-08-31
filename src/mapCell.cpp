#include "an_dynamic_planner/mapCell.h"
//#include "mapCell.h"

mapCell::mapCell(){}
mapCell::mapCell(carState state, int l, int t, int s){
	this->state = state;
	this->l = l;
	this->t = t;
	this->s = s;
	this->x = state.x;
	this->y = state.y;

	g = std::numeric_limits<int>::max();
	v = std::numeric_limits<int>::max();
	f = std::numeric_limits<int>::max();
	h = std::numeric_limits<int>::max();
}


double mapCell::getH(){
	return h;
}

mapCell::carState::carState(double x, double y, double theta, double t){
	this->x = x;
	this->y = y;
	this->theta = theta;
	this->t = t;
}
mapCell::carState::carState(){}

int mapCell::compareF(mapCell* b){
	if(f < b->f){
		return -1;
	}else if(f == b->f){
		if(g > b->g){
			return -1;
		}else if( g == b->g){
			return 0;
		}else{
			return 1;
		}
	}else{
		return 1;
	}
}