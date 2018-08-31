#include "an_dynamic_planner/planner.h"

int main(int argc, char** argv){
	//Turn on DEBUG
//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
  // ros::console::notifyLoggerLevelsChanged();
//}
	ros::init(argc, argv, "planner");
	ROS_DEBUG("[planner] Starting");
	Planner_class planner;
	if(planner.Init()){
		ROS_DEBUG("[planner] Entering loop");

		planner.Loop();
	}
}

void Planner_class::Loop(){
	int loop_rate_ = 150;
	ros::Rate loop(loop_rate_);
	while(ros::ok()&&new_plan_){
		if(new_plan_ && pose_received_ && goal_received_ && obstacle_received_ && map_received_){
			ROS_WARN("[planner] Started planning");
			new_plan_ = false;
			Planner_class::plan();
			ROS_WARN("[planner] Finished planning");
		}
		ros::spinOnce();
		loop.sleep();
	}
}
void Planner_class::plan(){
	pathPlanner traj_planner(&myData);
	//traj_planner.test();
	ros::Duration d(5.0);
	an_messages::trajectory traj = traj_planner.generateTraj();
	//ROS_WARN("[Traj]Fist Point is (%f, %f), final point is (%f, %f)", traj.traj[0].position.x, traj.traj[0].position.y, traj.traj[2345].position.x, traj.traj[2345].position.y);
	ros::Publisher traj_pub = nh.advertise<an_messages::trajectory>("planner_trajectory", 1, true);
	ros::Subscriber traj_sub = nh.subscribe("planner_trajectory",1, &Planner_class::traj_sub_callback, this);
	d.sleep();
	traj_pub.publish(traj);

	ros::spinOnce();


}

void Planner_class::traj_sub_callback(const an_messages::trajectory& msg){
	//ROS_DEBUG("[TrajCallback]");
	int points = 100;
	for(int i = 0 ; i < points; i++){
		//ROS_DEBUG("[TrajCallback] %d: x= %0.4f y=%0.4f", i, msg.traj[i].position.x, msg.traj[i].position.y);
	}
	
}


bool Planner_class::Init(){
	new_plan_ = true;
	loadMP();
	ROS_DEBUG("[planner] Entering Init");
	ros::Rate r(100);
	//All subcriber
	ros::Subscriber pose_sub = nh.subscribe("pose", 1, &Planner_class::pose_sub_callback, this);
	ros::Subscriber goal_sub = nh.subscribe("goal", 1, &Planner_class::goal_sub_callback, this);
	ros::Subscriber obstacle_sub = nh.subscribe("obstacles", 1, &Planner_class::obstacle_sub_callback, this);
	ros::Subscriber map_sub = nh.subscribe("lanes", 1, &Planner_class::map_sub_callback, this);
	while(ros::ok() && (!pose_received_ || !goal_received_ || !obstacle_received_ || !map_received_)){
		if(pose_received_) pose_sub.shutdown();
		if(goal_received_) goal_sub.shutdown();
		if(obstacle_received_) obstacle_sub.shutdown();
		if(map_received_) map_sub.shutdown();
		ros::spinOnce();
		r.sleep();
	}
	return true;
}

Planner_class::Planner_class(){
	new_plan_ = false;
	pose_received_ = false;
	goal_received_ = false;
	obstacle_received_ = false;
	map_received_ = false;
}

void Planner_class::pose_sub_callback(const geometry_msgs::PoseStamped& msg){
	myData.global_start.x = msg.pose.position.x;
	myData.global_start.y = msg.pose.position.y;
	myData.global_start.theta = tf::getYaw(msg.pose.orientation);
	ROS_DEBUG("[Global_Start] x:%lf  y:%lf theta:%lf", myData.global_start.x, myData.global_start.y, myData.global_start.theta);
	pose_received_ = true;
}

void Planner_class::goal_sub_callback(const geometry_msgs::PoseStamped& msg){
	myData.global_goal.x = msg.pose.position.x;
	myData.global_goal.y = msg.pose.position.y;
	myData.global_goal.theta = tf::getYaw(msg.pose.orientation);
	ROS_DEBUG("[Global_Goal] x:%lf  y:%lf theta:%lf", myData.global_goal.x, myData.global_goal.y, myData.global_goal.theta);
	goal_received_ = true;
}
void Planner_class::obstacle_sub_callback(const an_messages::obstacles& msg){
	myData.length = msg.obs[0].length;
	myData.width = msg.obs[0].width;
	myData.outer_radius = msg.obs[0].outer_radius;
	myData.inner_radius = msg.obs[0].inner_radius;
	myData.startTime = msg.obs[0].path[0].traj[0].header.stamp;
	for(int o = 0; o < msg.obs.size(); o++){
		int path_num = msg.obs[o].path.size();
		allInfoClass::obstacle ob(path_num);
		//cout << "path_num" << path_num << endl;
		while(path_num > 0){
			double start_time = msg.obs[o].path[path_num-1].traj[0].header.stamp.toSec();
			for(int i = 0; i < msg.obs[o].path[path_num-1].traj.size(); i++){
				mapCell::carState obState;
				obState.x = msg.obs[o].path[path_num-1].traj[i].position.x;
				obState.y = msg.obs[o].path[path_num-1].traj[i].position.y;
				obState.theta = msg.obs[o].path[path_num-1].traj[i].position.theta;
				obState.t = round((msg.obs[o].path[path_num-1].traj[i].header.stamp.toSec() - start_time)*100)/100.0;
				ob.path[path_num-1].push_back(obState);

				divideObsByTime(obState);
			}
			path_num--;
		}
		myData.obs.push_back(ob);
		//cout << ob.x << ' ' << ob.y << ' ' << ob.theta << endl;
		ROS_DEBUG("[Obstacle] Push obstacle %d.", o);
	}
	obstacle_received_ = true;
	ROS_DEBUG("[Obstacle] Received.");
}
void Planner_class::divideObsByTime(mapCell::carState obState){
	double t = obState.t;
	if(myData.allObsEveryTimeStep.find(t)==myData.allObsEveryTimeStep.end()){//if haven't push any obs at this time step
		vector<mapCell::carState> obs_now;
		obs_now.push_back(obState);
		myData.allObsEveryTimeStep.insert(pair<double, vector<mapCell::carState> >(t, obs_now));
		//ROS_DEBUG("Push a vector into step obs");
		//cout << "New vector obs at time " << t << endl;
	}else{
		myData.allObsEveryTimeStep.at(t).push_back(obState);
		//ROS_DEBUG("Push a ob into a vector");
	}
}
void Planner_class::map_sub_callback(const an_messages::lanes& msg){
	an_messages::lanes map_msg = msg;
	int station_num = 2001;//total stations
	int track_num = 5;//track num for each lane
	double cell_l = 1.0;//cell size in x direction
	double cell_w = 0.74;//cell size in y direction
	myData.initMap(map_msg, station_num, track_num, cell_w, cell_l);
	map_received_ = true;
	ROS_DEBUG("[Map] Received.");

}

bool Planner_class::loadMP(){
	while(!ros::param::has("MPRIM_FILE")){
		ROS_DEBUG("[MP]sleeping while waiting");
		ros::Duration(0.1).sleep();
	}
	string fname;
	ros::param::get("/MPRIM_FILE",fname);
	ROS_DEBUG("[MP] %s", fname.c_str());

	FILE* fp = fopen(fname.c_str(),"r");
	if(fp==NULL){
		ROS_DEBUG("[MP] No motion primitve file! Exiting!!");
		return false;
	}
	//Straight, right, left
	for(int i = 0; i < 3; i++){
		int num;
		char dir[100];
		fscanf(fp, "%s %i", dir, &num);
		ROS_DEBUG("[MP] %s MP has %d points.", dir, num);
		vector<mapCell::carState>* vector_ptr ;
		if(strcmp(dir, "straight") == 0){
			vector_ptr = &(myData.straight_MP);
		}else if(strcmp(dir, "left") == 0){
			vector_ptr = &(myData.left_MP);
		}else{
			vector_ptr = &(myData.right_MP);
		}
		for(int j = 0; j < num; j++){
			double xj, yj, thetaj, tj;
			fscanf(fp, "%lf %lf %lf %lf", &xj, &yj, &thetaj, &tj);
			mapCell::carState temp(xj, yj, thetaj, tj);
			vector_ptr->push_back(temp);
		}
		ROS_DEBUG("[MP] Finish loading %s MP.", dir);
	}
	//MP cost
	myData.stCost = myData.straight_MP.back().x;//straight cost
	myData.turnCost = myData.right_MP.back().t*25.0;//turn cost
	return true;
}
