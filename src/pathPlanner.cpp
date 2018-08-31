#include "an_dynamic_planner/pathPlanner.h"

pathPlanner::pathPlanner(allInfoClass* ptr){
	data_ptr = ptr;
	//data_ptr->global_goal = &(ptr->global_goal);
	//data_ptr->global_start = &(ptr->global_start);
	lane_w = data_ptr->lane_w;
	cell_w = data_ptr->cell_w;
	cell_l = data_ptr->cell_l;
	getData_ = true;
	//arrived_ = false;
}
pathPlanner::pathPlanner(){};
//ARA*
an_messages::trajectory pathPlanner::generateTraj(ros::Duration d){
	ROS_WARN("[PathPlanner] Start path planner!");
	an_messages::trajectory traj;
	if(!getData_){
		ROS_WARN("[PathPlanner] Fail to get data we need!");
		return traj;
	}
	//Start Cell And Goal Cell
	mapCell::carState start_state = data_ptr->global_start;
	mapCell::carState goal_state = data_ptr->global_goal;

	mapCell* start_cell = positionToCell(start_state.x, start_state.y);
	mapCell* goal_cell = positionToCell(goal_state.x, goal_state.y);
	if(start_cell == NULL || goal_cell == NULL){
		ROS_ERROR("[PathPlanner] Start Or Goal position is not in center of any cell!");
		return traj;
	}
	//Prepair for ARA*
	double eps = 3.0;//Start epsilon.
	double eps_dicrease = 1.0;
	double eps_prime = eps;
	start_cell->g = 0;
	MinHeap opened;
	vector<mapCell*> incons;
	setH(start_cell);
	start_cell->f = start_cell->h*eps;
	opened.push(start_cell);
	//ARA*
	int count = 1;//ARA search time
	if(!ARASearch(start_cell, goal_cell, &opened, &incons, eps, count)) return traj;
	while(eps_prime > 1){
		eps -=eps_dicrease;//Direase epsilon
		combineOpenedAndIncons(&opened, &incons);
		incons.empty();//clean up incons vector
		updateWithNewEps(&opened, eps);
		count++;
		if(!ARASearch(opened.peekTop(), goal_cell, &opened, &incons, eps, count)) return traj;
		eps_prime = callEpsPrime(eps, &opened, goal_cell);
		ROS_DEBUG("[EpsPrime] new_eps=%0.4f eps_prime=%0.4f", eps, eps_prime);
		
	}
	traj = backtrackTraj(goal_cell, start_cell, d);
	return traj;
}
double pathPlanner::callEpsPrime(double eps, MinHeap* opened, mapCell* goal_cell){
	if(opened->heap_size == 0){
		return 1;
	}
	return min(eps, goal_cell->f/opened->peekTop()->f);
}
//Update f value of all member in opened
void pathPlanner::updateWithNewEps(MinHeap* opened, double new_eps){
	ROS_DEBUG("Update opened with new eps %0.2f", new_eps);
	if(opened->heap_size == 0) return;
	for(int i = 0; i < opened->heap_size; i++){
		//cout<<"before:" << opened->heap[i]->f << '\n';
		(*opened).heap.at(i)->f = (*opened).heap.at(i)->g+new_eps*(*opened).heap.at(i)->h;//Update f value with new epsilon
		opened->update(opened->heap.at(i));
		//cout << "update" << opened->heap[i]->f <<'\n';
	}
	//Print out updated opened heap
	MinHeap temp;
	for( int t = 0; t < opened->heap_size; t++){
		temp.push(opened->heap[t]);
	}
	//int layer = 1;
	//int i = 0;
	//int j = layer;
	//ROS_DEBUG("[Opened] My opened list %d elements", opened->heap_size);
	//while(i < opened->heap_size){
		//while(j > 0 && i <opened->heap_size){
			//cout << opened->heap[i]->f << ' ';
			//i++;
			//j--;
		//}
		//layer *=2;
		//j = layer;
		//cout << '\n';
	//}
	//ROS_DEBUG("[OpenedTemp]");
	//i = 0;
	//layer = 1; 
	//j = layer;
	//while(i <temp.heap_size){
		//while(j > 0 && i <temp.heap_size){
			//cout << temp.heap[i]->f << ' ';
			//i++;
			//j--;
		//}
		//layer *=2;
		//j = layer;
		//cout << '\n';
	//}
}
//ARA* return a vector of MP
bool pathPlanner::ARASearch(mapCell* start_cell, mapCell* goal_cell, MinHeap* opened, vector<mapCell*>* incons,  double eps, int count){
	mapCell* cur_cell = start_cell;
	ROS_DEBUG("[ARA] Start %dth ARA search, from (l%d, t%d, s%d).", count, cur_cell->l, cur_cell->t, cur_cell->s);
	vector<mapCell*> closed;
	double stCost = data_ptr->stCost;
	double turnCost = data_ptr->turnCost;
	ros::Time start_time = ros::Time::now();
	while(!opened->isEmpty() && goal_cell->g > cur_cell->f){
		//ROS_WARN("[ARA] Start ARA search");
		//ROS_INFO("[ARA] %d Now Search is at (l%d, t%d, s%d).",count, cur_cell->l, cur_cell->t, cur_cell->s);
		//Put the min-f value cell into closed list
		//ROS_INFO("[ARA] Expend this cell");
		opened->popTop();
		//cout << "pop out from opened and push into closed " << cur_cell  << " : " << cur_cell->f<<'(' << cur_cell->l<<cur_cell->t << cur_cell->s << '\n';
		cur_cell->v = cur_cell->g;
		//cout << "Push it into closed " << cur_cell  << " : " << cur_cell->f<< '(' << succ[i]->l<<succ[i]->t << succ[i]->s << '\n';
		closed.push_back(cur_cell);
		//generate min-f value cell's successors
		vector<mapCell*> succ = generateSuccessors(cur_cell);
		//Push valid successors into opened list or incons list.
		for(int i = 0; i < succ.size(); i++){
			//cost from current cell to successor
			double cost = MPCost(cur_cell, succ[i]);
			//If successor's g could be update
			if(succ[i]->g > cur_cell->g+cost){
				succ[i]->g = cur_cell->g+cost;
				if(!isInVector(succ[i], &closed)){//If it's not in closed list
					succ[i]->f = succ[i]->g+eps*succ[i]->h;
					succ[i]->parent = cur_cell;
					if(!opened->find(succ[i])){//If it's not in opened list
						opened->push(succ[i]);
					//cout << "push into opened " << succ[i]  << " : " << succ[i]->f<<'(' << succ[i]->l<<succ[i]->t << succ[i]->s << '\n';
					}else{
						opened->update(succ[i]);
						//cout << "update in opened " << succ[i]  << " : " << succ[i]->f<<'(' << succ[i]->l<<succ[i]->t << succ[i]->s << '\n';
					}
					//int layer = 1;
					//int o = 0;
					//int j = layer;
					//ROS_DEBUG("[Opened] My opened list %d elements", opened->heap_size);
					//while(o < opened->heap_size){
						//while(j > 0 && o <opened->heap_size){
						//	cout << opened->heap[o]->f << ' ';
						//	o++;
						//	j--;
						//}
						//layer *=2;
						//j = layer;
						//cout << '\n';
					//}
				}else{//If it's in closed list
					ROS_DEBUG("[Push into incons] %0.4f", succ[i]->f);
					incons->push_back(succ[i]);
				}
			}
		}
		count++;
		cur_cell = opened->peekTop();
		if(ros::Time::now()-start_time > ros::Duration(30)){
			ROS_DEBUG("[ARA] Time out! Can't find goal!");
			return false;
		}
	}
	ROS_DEBUG("[ARA] Finish ARA!");
	return  true;
}

//Didn't finish
an_messages::trajectory pathPlanner::backtrackTraj(mapCell* goal_cell, mapCell* start_cell, ros::Duration d){
	ROS_DEBUG("[Traj] Generate trajectory from goal to start.");
	an_messages::trajectory traj;
	//Genertate trajectory with MP, form goal to start
	mapCell* cur_cell = goal_cell;
	mapCell* parent_cell = goal_cell->parent;
	mapCell* pre = goal_cell->parent;
	vector<vector<mapCell::carState>*> MPTraj;
	int pointNum = 1;
	int iter = 0;
	while(cur_cell!=start_cell){
		if(parent_cell->l == cur_cell->l){//Case 1 go straight
			MPTraj.push_back(&(data_ptr->straight_MP));
			pointNum += data_ptr->straight_MP.size()-1;
			//ROS_DEBUG("[Traj] Generate trajectory from goal to start.");
		}else if(parent_cell->l > cur_cell->l){//Case 2 turn right
			MPTraj.push_back(&(data_ptr->right_MP));
			pointNum += data_ptr->right_MP.size()-1;
			//ROS_DEBUG("[Traj] Generate trajectory from goal to start.");
		}else{
			MPTraj.push_back(&(data_ptr->left_MP));//Case3 turn left
			pointNum += data_ptr->left_MP.size()-1;
			//ROS_DEBUG("[Traj] Generate trajectory from goal to start.");
		}
		pre = cur_cell;
		cur_cell = parent_cell;
		parent_cell = cur_cell->parent;
		if(cur_cell == pre){
			ROS_DEBUG("[ARA]Error: Loop when persure traj from goal to start, at %d & %d", iter, iter-1);
			return traj;
		}
		iter++;
	}
	ROS_DEBUG("%d",pointNum);
	traj.traj.resize(pointNum);
	ros::Time start_time = data_ptr->startTime;
	traj.header.frame_id = "/map";
	double start_x = start_cell->x;
	double start_y = start_cell->y;
	double start_theta = 0;
	vector<mapCell::carState>* MP;
	traj.traj[0].header.stamp = start_time+ros::Duration(0);
	traj.traj[0].header.frame_id = "/map";
	traj.traj[0].position.x = start_x;
	traj.traj[0].position.y = start_y;
	traj.traj[0].position.theta = start_theta;
	traj.traj[0].velocity.linear.x = 25.0*cos(0);
	traj.traj[0].velocity.linear.y = 25.0*sin(0);
	traj.traj[0].velocity.linear.z = 0.0;
	int hitr = 0;//print out first 5 points of each MP
	int idx = 1;
	for(int i = MPTraj.size()-1; i >=0; i--){
		MP = MPTraj[i];
		//drop the first element in MP array
		for(int j = 1; j < MP->size(); j++){
			traj.traj[idx].header.stamp = start_time+ros::Duration(MP->at(j).t);
			traj.traj[idx].header.frame_id = "/map";
			traj.traj[idx].position.x = start_x + MP->at(j).x;
			traj.traj[idx].position.y = start_y + MP->at(j).y;
			traj.traj[idx].position.theta = start_theta+MP->at(j).theta;
			traj.traj[idx].velocity.linear.x = 25.0*cos(MP->at(j).theta);
			traj.traj[idx].velocity.linear.y = 25.0*sin(MP->at(j).theta);
			traj.traj[idx].velocity.linear.z = 0.0;
			//Print the first 5 points of this MP
			if (hitr < 5) {
        		ROS_DEBUG("x:%f y:%f th:%f vel:%f", traj.traj[idx].position.x, traj.traj[idx].position.y, traj.traj[idx].position.theta, traj.traj[idx].velocity.linear.x);
    		}
    		idx++;
    		hitr++;
		}
		hitr = 0;
		ROS_DEBUG("[BackTrackTraj]Finish Push %dth MP to Traj Message.", i);
		start_time = traj.traj[idx-1].header.stamp;
		start_x += MP->back().x;
		start_y += MP->back().y;
		start_theta += MP->back().theta;
	}
	return traj;
}
//Combine Poened list and Incons list
void pathPlanner::combineOpenedAndIncons(MinHeap* opened, vector<mapCell*>* incons){
	if(incons == NULL || incons->size() == 0){
		ROS_DEBUG("[Combine OP&INC] Incons is empty.");
		return;
	}
	ROS_WARN("[Combine OP&INC] Combine opened and Incons.");
	for(int i = 0; i < incons->size(); i++){
		opened->push(incons->at(i));
	}
	//cout << "\nOpen:";
	//for(int i = 0; i < opened->heap_size; i++){
	//cout << opened->heap.at(i)  << ' ';
	//}
	//cout << "\nIncons:";
	//for(int i = 0; i < incons->size(); i++){
	//	cout << incons->at(i) << ' ';
	//}
}
//Cost of MP
double pathPlanner::MPCost(mapCell* cur, mapCell* next){
	if(cur->l-next->l == 0){
		//cout << "stCost" << data_ptr->stCost;
		return data_ptr->stCost;
	}else{
		//cout << " turnCost" << data_ptr->turnCost;
		return data_ptr->turnCost;
	}
}
//Generate successors of a given cell
vector<mapCell*> pathPlanner::generateSuccessors(mapCell* cur_cell){
	//Check if out of targer lane;
	ROS_DEBUG("[GetSucc] Generate (l%d t%d s%d)'s successors", cur_cell->l, cur_cell->t,cur_cell->s);
	mapCell::carState cur_state = cur_cell->state;
	int curL = cur_cell->l;
	vector<mapCell*> successors;
	if(curL == 0){//In the most right lane, can't turn right.
		if(!checkHit(&cur_state, &(data_ptr->left_MP))){
			ROS_DEBUG("[GetSucc] Can left turn.");
			putIntoSuccessors(cur_cell, &(data_ptr->left_MP), &successors);
		}
		if(!checkHit(&cur_state, &(data_ptr->straight_MP))){
			ROS_DEBUG("[GetSucc] Can go straight.");
			putIntoSuccessors(cur_cell, &(data_ptr->straight_MP), &successors);
		}
	}else if(curL == data_ptr->edges.size()-1){//In the most left lane, can't turn left.
		if(!checkHit(&cur_state, &(data_ptr->right_MP))){
			ROS_DEBUG("[GetSucc] Can right turn.");
			putIntoSuccessors(cur_cell, &(data_ptr->right_MP), &successors);
		}
		if(!checkHit(&cur_state, &(data_ptr->straight_MP))){
			ROS_DEBUG("[GetSucc] Can go straight.");
			putIntoSuccessors(cur_cell, &(data_ptr->straight_MP), &successors);
		}
	}else{//In middle lanes, can turn to any direction
		if(!checkHit(&cur_state, &(data_ptr->left_MP))){
			ROS_DEBUG("[GetSucc] Can left turn.");
			putIntoSuccessors(cur_cell, &(data_ptr->left_MP), &successors);
		}
		if(!checkHit(&cur_state, &(data_ptr->right_MP))){
			ROS_DEBUG("[GetSucc] Can right turn.");
			putIntoSuccessors(cur_cell, &(data_ptr->right_MP), &successors);
		}
		if(!checkHit(&cur_state, &(data_ptr->straight_MP))){
			ROS_DEBUG("[GetSucc] Can go straight.");
			putIntoSuccessors(cur_cell, &(data_ptr->straight_MP), &successors);
		}
	}
	return successors;
}
//Put next cell into Successors after run along a MP, return next cell pointer.
mapCell* pathPlanner::putIntoSuccessors(mapCell* cur_cell, vector<mapCell::carState>* MP, vector<mapCell*>* successors){
	mapCell::carState next_state;
	next_state.x = cur_cell->state.x+MP->back().x;
	next_state.y = cur_cell->state.y+MP->back().y;
	next_state.theta = cur_cell->state.theta+MP->back().theta;
	next_state.t = cur_cell->state.t+MP->back().t;
	mapCell* next_cell = positionToCell(next_state.x, next_state.y);
	//If x y isn't center of a cell, return NULL.
	if(next_cell == NULL){
		return NULL;
	}
	//Update cell state
	//Put into successors
	next_cell->state = next_state;//update state of next cell
	next_cell->parent = cur_cell;
	if(next_cell->h == numeric_limits<int>::max()){
		setH(next_cell);
	}
	successors->push_back(next_cell);
	ROS_DEBUG("Put (l%d t%d s%d) into successors", next_cell->l, next_cell->t, next_cell->s);
	return next_cell;
}
bool pathPlanner::checkHit(mapCell::carState* start, vector<mapCell::carState>* dir_MP){
	double startTime = start->t;
	double length = data_ptr->length;
	double width = data_ptr->width;
	double inner_r = data_ptr->inner_radius;
	double outer_r = data_ptr->outer_radius;
	double cur_x, cur_y, cur_theta, cur_t;//cur state in MP
	for(int mi = 0; mi < dir_MP->size(); mi++){//MP point
		cur_t = startTime+dir_MP->at(mi).t;
		double remain = fmod(cur_t, 0.05);
		if(remain == 0){
			;
		}else if(remain < 0.025){
			cur_t = cur_t - remain;
		}else{
			cur_t = cur_t - remain + 0.05;
		}
		cur_t = round(cur_t*100)/100.0;
		cur_x = start->x+dir_MP->at(mi).x;
		cur_y = start->y+dir_MP->at(mi).y;
		cur_theta = dir_MP->at(mi).theta;
		if(data_ptr->allObsEveryTimeStep.find(cur_t) == data_ptr->allObsEveryTimeStep.end()){
			ROS_DEBUG("[CheckHit]Error:%f is not in my allObsEveryTimeStep!", cur_t);
		}
		vector<mapCell::carState>* obs_ptr = &(data_ptr->allObsEveryTimeStep.at(cur_t));
		//ROS_DEBUG("[CheckHit] Now Car at (x%0.4f y%0.4f t%0.4f)",cur_x, cur_y, cur_theta);
		double o_c_dist;//Distance between obstacle and car
		//cout << obs_ptr->size();
		for(int oi = 0; oi < obs_ptr->size(); oi++){//obstacle iterator
			//ROS_DEBUG("[CheckHit] Ob%d at (x%0.4f y%0.4f t%0.4f).", oi, obs_ptr->at(oi).x, obs_ptr->at(oi).y, obs_ptr->at(oi).theta);
			o_c_dist = sqrt(pow(cur_x-obs_ptr->at(oi).x,2)+pow(cur_y-obs_ptr->at(oi).y,2));
			if(o_c_dist > 2.0*outer_r){
				//ROS_DEBUG("[CheckHit] Case1:Won't Hit! dist=%0.4f.",o_c_dist);
				continue;//case 1 continue check next obsticale
			}
			if(o_c_dist < 2.0*inner_r){
				//ROS_DEBUG("[CheckHit] Case2:Hit! dist=%0.4f.",o_c_dist);
				//ROS_DEBUG("[HitOb] At (x%0.4f y%0.4f t%0.4f)", obs_ptr->at(oi).x, obs_ptr->at(oi).y, obs_ptr->at(oi).theta);
			 	return true;// case 2
			}
			//case 3: Using 3 circles cover the car and the obstacle, pick 2 circles in different object
			//Ckech if they will hit, 9 groups
			//ROS_DEBUG("[CheckHit] Case3:My hit! dist=%0.4f.",o_c_dist);
			double l_offset = length/3.0;
			double c_x = cur_x-l_offset*cos(cur_theta);//car circle position
			double c_y = cur_y-l_offset*sin(cur_theta);
			for(int ci = 0; ci < 3; ci++){
				if(checkHitHelper(c_x, c_y, &(obs_ptr->at(oi)))){
					return true;
				} 
				c_x += l_offset*cos(cur_theta);
				c_y += l_offset*sin(cur_theta);
			}
		}
	}
	
	return false;
}
//Input a circle in car and an obstacle, check if they will hit.
//Hit return true, won't hit return false.
bool pathPlanner::checkHitHelper(double c_x, double c_y, mapCell::carState* ob){
	double o_theta = ob->theta;
	double l_offset = data_ptr->length/3.0;
	double o_x = ob->x - l_offset*cos(o_theta);
	double o_y = ob->y - l_offset*sin(o_theta);
	double hitDist = 2.0*sqrt(pow(data_ptr->length/6.0,2)+pow(data_ptr->width/2.0, 2));
	for(int i = 0; i < 3; i++){
		if(sqrt(pow(c_x-o_x, 2) + pow(c_y-o_y, 2)) <= hitDist){
			//ROS_DEBUG("[CheckHitHelper] Hit! CarCir (%0.4f %0.4f) VS ObCir (%0.4f %0.4f)", c_x, c_y,o_x, o_y);
			//ROS_DEBUG("[HitOb] At (x%0.4f y%0.4f t%0.4f)", ob->x, ob->y, ob->theta);
			 	
			return true;
		}
		o_x += l_offset*cos(o_theta);
		o_y += l_offset*sin(o_theta);
	}
	//ROS_DEBUG("[CheckHitHelper] Won't Hit! CarCir (%0.4f %0.4f) VS Ob (x%0.4f y%0.4f t%0.4f)", c_x, c_y, ob->x, ob->x, ob->theta);
	return false;
}
void pathPlanner::setH(mapCell* cell_ptr){
	cell_ptr->h = sqrt(pow(abs(data_ptr->global_goal.x-cell_ptr->x),2)+pow(abs(data_ptr->global_goal.y-cell_ptr->y),2));
	//ROS_DEBUG("[Set_H]POS(%0.4f, %0.4f) h= %lf", cell_ptr->x, cell_ptr->y, cell_ptr->h );
}

mapCell* pathPlanner::positionToCell(double x, double y){
	int l = (int)((y-data_ptr->edges[0].left_edge)/lane_w);
	if(y < data_ptr->edges[0].left_edge || y > data_ptr->edges.back().right_edge){
		if(y == data_ptr->edges.back().right_edge && l == data_ptr->edges.size()){
			l = l-1;
		}else{
			//ROS_DEBUG("[PositionToCell] Position (%0.4f, %0.4f) is out of map!", x, y);
			return NULL;
		}	
	}
	int t = (int)((y-data_ptr->edges[l].left_edge)/cell_w);
	int s = (int)((x+cell_l/2.0)/cell_l);
	mapCell* temp = data_ptr->getCell(l,t,s);
	if(temp->x != x || temp->y != y){
		ROS_DEBUG("[PosToCell] ERROR: Input position is not cell center");
		return NULL;
	}
	//ROS_DEBUG("[PosToCell] POS(%0.4f, %0.4f) -> C(%d %d %d)", x, y, l, t, s);
	return data_ptr->getCell(l, t, s);
}
//test useless
void pathPlanner::test(){
	mapCell* c = positionToCell(data_ptr->global_goal.x, data_ptr->global_goal.y);
	cout << "test" << c->l << c->t << c->s <<'\n';
}
//Check if cell in a vector
bool pathPlanner::isInVector(mapCell* cell, vector<mapCell*>* vec){
	if(vec == NULL) {
		ROS_DEBUG("[IsMember] Is Empty");
		return false;
	}
	for(int i = 0; i < vec->size(); i++){
		if(vec->at(i) == cell){
			return true;
		}
	}
	return false;
}