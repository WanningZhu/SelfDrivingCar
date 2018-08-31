/* Copyright (C) 2018 RobotWits, LLC - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Written by Jonathan Butzke <instructor@robotwits.com>, January, 2018
 */

#include "an_obstacles/obstacles.h"


std::vector<an_messages::obstacle> ReadObstacleFile(ros::Time StartTime) {
  std::vector<an_messages::obstacle> obs;
  int numobs = 0;

  std::string fname;
  while (!ros::param::has("OBS_FILE")) {
    ROS_WARN("[obstacle] sleeping while waiting");
    ros::Duration(0.1).sleep();
  }

  ros::param::get("/OBS_FILE", fname);
  ROS_DEBUG("[obstacle] %s", fname.c_str() );

  FILE* fp = fopen(fname.c_str(), "r");
  if (fp == NULL) {
    ROS_WARN("[obstacle] No obstacle file!  Exiting!!");
    obs.clear();
    return obs;
  }

  int sz = fscanf(fp, "%i", &numobs);
  if (sz == 0) {
    ROS_WARN("[obstacle] Error with obstacle file: missing number of obstacles");
    obs.clear();
    return obs;
  }

  obs.resize(numobs);

  for (int oidx =-0; oidx < numobs; oidx++) {
    int numpaths=0;
    sz = fscanf(fp, "%i %lf %lf %lf %i", &obs[oidx].id, &obs[oidx].width, &obs[oidx].length, &obs[oidx].height, &numpaths);
    if (sz != 5) {
      ROS_WARN("[obstacle] Error with obstacle file: missing some of id, width, length, or number of paths");
      obs.clear();
      return obs;
    }
    ROS_WARN("[obstacle] %i %lf %lf %lf %i", obs[oidx].id, obs[oidx].width, obs[oidx].length, obs[oidx].height, numpaths);
    obs[oidx].header.frame_id = "/map";
    obs[oidx].header.stamp = ros::Time::now();
    obs[oidx].header.seq = oidx;
    obs[oidx].outer_radius = sqrt(obs[oidx].length*obs[oidx].length/4 + obs[oidx].width*obs[oidx].width/4);
    obs[oidx].inner_radius = obs[oidx].width/2;

    obs[oidx].path.resize(numpaths);
    obs[oidx].probability.resize(numpaths);
    for (int pidx=0; pidx < numpaths; pidx++) {
      geometry_msgs::Pose2D pose;
      int numtraj;
      sz = fscanf(fp, "%i %lf", &numtraj, &obs[oidx].probability[pidx]);
      if (sz != 2) {
        ROS_WARN("[obstacle] Error with obstacle file : missing number of trajectories or a probability");
        obs.clear();
        return obs;
      }
      ROS_DEBUG("[obstacle] %i %lf", numtraj, obs[oidx].probability[pidx]);
      //obs[oidx].path[pidx].traj.resize(numtraj);  Would add in for static sizing array
      for (int tidx=0; tidx < numtraj; tidx++) {
        double secsfromstart=0;
        sz = fscanf(fp, "%lf %lf %lf %lf", &pose.x, &pose.y, &pose.theta, &secsfromstart);
        if (sz != 4) {
          ROS_WARN("[obstacle] Error with obstacle file: error in the pose or time");
          obs.clear();
          return obs;
        }
        ROS_DEBUG("[obstacle] %lf %lf %lf %lf", pose.x, pose.y, pose.theta, secsfromstart);
        an_messages::traj_pt tp;
        tp.position = pose;
        tp.header.stamp = StartTime + ros::Duration(secsfromstart + TIME_DELAY);
        obs[oidx].path[pidx].traj.push_back(tp);
      }
    }
  }
  fclose(fp);
  return obs;
}



int main( int argc, char** argv )  {
  ros::init(argc, argv, "obstacles");
  ros::NodeHandle nh;
  ros::Rate r(30);


  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_markerarray", 5, false);
  ros::Publisher sidemarker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 5, false);
  ros::Publisher obs_pub = nh.advertise<an_messages::obstacles>("obstacles", 5, true);

  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 5, true);
  ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("goal", 5, true);

  ros::Duration(1).sleep();

  ros::Time StartTime = ros::Time::now();


  std::vector<an_messages::obstacle> obs = ReadObstacleFile(StartTime);
  visualization_msgs::MarkerArray markerarray;
  visualization_msgs::Marker sidemarker, marker;

  sidemarker.header.stamp = ros::Time::now();
  sidemarker.header.frame_id = "/map";
  sidemarker.type = visualization_msgs::Marker::CUBE_LIST;
  sidemarker.ns = "side_marker";
  sidemarker.action = visualization_msgs::Marker::ADD;
  sidemarker.lifetime = ros::Duration();
  sidemarker.color.r = 1.0f;//0.4f;
  sidemarker.color.g = .5f;//0.0f;
  sidemarker.color.b = 0;//1.0f;
  sidemarker.color.a = 1;
  sidemarker.scale.x = 0.1;//obs[oidx].length;
  sidemarker.scale.y = 1;//obs[oidx].width;
  sidemarker.scale.z = 0.01;//obs[oidx].height;
  sidemarker.id = 666;

  sidemarker.pose.position.x = 0;
  sidemarker.pose.position.y = 0;
  sidemarker.pose.position.z = 0;
  sidemarker.pose.orientation.x = 0;
  sidemarker.pose.orientation.y = 0;
  sidemarker.pose.orientation.z = 0;
  sidemarker.pose.orientation.w = 1;

    for (int idx = 0; idx < 100; idx++)  {
          geometry_msgs::Point p;
                  p.x = idx * 20;
                  p.y = -3.7/2 -1;
                  p.z = 0;
                  sidemarker.points.push_back(p);
                 p.y = 3.7/2 + 3.7*2 + 1;
                 sidemarker.points.push_back(p);
    }

    sidemarker_pub.publish(sidemarker);

  marker.header.frame_id = "/map";
 // marker.type = visualization_msgs::Marker::CUBE;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.mesh_use_embedded_materials = true;
  marker.mesh_resource = "package://an_obstacles/config/DASA.dae";


  marker.ns = "obstacles";
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration();
  marker.color.r = 0.8;//0.4f;
  marker.color.g = 0.5;//0.0f;
  marker.color.b = 0;//1.0f;
  marker.color.a = 1.0;//1.0;

  std::vector<int> trajptr;  //TODO:  this assumes only a single trajectory per vehicle
  trajptr.resize(obs.size());
  for (int tidx=0; tidx < trajptr.size(); tidx++) {
    trajptr[tidx] = 0;
  }

// use param to set alternate trajectory for vehicle 0
  float intent;
  nh.param("/driver_intends_to_change_lanes", intent, (float)0.0);
  ROS_WARN("Driver intention is %f", intent);

  for (int o=0; o < obs.size(); o++) {
    for (int p=0; p < obs[o].path.size(); p++) {
      for (int t=0; t < obs[o].path[p].traj.size(); t++) {
        ROS_DEBUG("[obstacle] pt(%i, %i, %i) is x:%f y:%f, th:%f, t:%i %i", o, p, t, obs[o].path[p].traj[t].position.x, obs[o].path[p].traj[t].position.y, obs[o].path[p].traj[t].position.theta, obs[o].path[p].traj[t].header.stamp.sec, obs[o].path[p].traj[t].header.stamp.nsec);
      }
    }
  }
  int counter=0;
  int obs_rate =0;

  geometry_msgs::PoseStamped goal;
  geometry_msgs::PoseStamped start;

  start.pose.position.x = 0;
  start.pose.position.y = 0;
  start.pose.position.z = 0;
  start.pose.orientation.x = 0;
  start.pose.orientation.y = 0;
  start.pose.orientation.z = 0;
  start.pose.orientation.w = 1;
  start.header.stamp = ros::Time::now();

  pose_pub.publish(start);
  goal = start;
  nh.param("/goalx", goal.pose.position.x, 1800.0);
  nh.param("/goaly", goal.pose.position.y, 7.4);
  goal_pub.publish(goal);


  while (ros::ok())   {
    markerarray.markers.clear();
    marker.header.stamp = ros::Time::now();
    for (int oidx = 0; oidx < obs.size(); oidx++) {
      marker.id = obs[oidx].id;

      marker.scale.x = 1;//obs[oidx].length;
      marker.scale.y = 1;//obs[oidx].width;
      marker.scale.z = 1;//obs[oidx].height;

      //   for (int pidx=0; pidx < obs[oidx].path.size(); pidx++) {  //TODO: add back in later
      int pidx;
      if (oidx==0) {
        pidx = intent;
      } else {
        pidx=0;
      }

      {
        if ((trajptr[oidx] != -1) && (marker.header.stamp > obs[oidx].path[pidx].traj[trajptr[oidx]].header.stamp)) {
          // (trajptr[oidx] < obs[oidx].path[pidx].traj.size()) &&
          marker.pose.position.x = obs[oidx].path[pidx].traj[trajptr[oidx]].position.x;
          marker.pose.position.y = obs[oidx].path[pidx].traj[trajptr[oidx]].position.y;
          marker.pose.position.z = 1.60;//obs[oidx].height/2;
          marker.pose.orientation = tf::createQuaternionMsgFromYaw(obs[oidx].path[pidx].traj[trajptr[oidx]].position.theta);

          markerarray.markers.push_back(marker);

          ROS_DEBUG("[obstacle] moving at x: %f idx: %i(%i)", marker.pose.position.x, trajptr[oidx], oidx);
          //obs_pub.publish(obs[oidx]);  //TODO: maybe update this as vehicles move

          trajptr[oidx]++;
          if (trajptr[oidx] >= obs[oidx].path[pidx].traj.size()) {
            trajptr[oidx] = -1;
          }
        }
      }
    }
//ROS_WARN("size %li", markerarray.markers.size() );
//if (markerarray.markers.size() > 0)  ROS_WARN("obs 0 x:%2.1f y: %2.1f time: %3.3f (%3.3f)", markerarray.markers[0].pose.position.x, markerarray.markers[0].pose.position.y, markerarray.markers[0].header.stamp.toSec(), obs[0].path[0].traj[trajptr[0]].header.stamp.toSec() );


    //     bool done = false;
    //     for (int i=0; i < trajptr.size(); i++) {
    //       if (trajptr[i] == -1) {
    //         done = true;
    //       }
    //     }
    //     if (done) {break;}
    marker_pub.publish(markerarray);
    obs_rate++;
    if (obs_rate > 15) {
      obs_rate = 0;
      an_messages::obstacles obs_list;
      obs_list.obs = obs;
      obs_list.header.stamp = ros::Time::now();
      obs_list.header.frame_id = "/map";
      obs_list.header.seq = counter++;
      obs_pub.publish(obs_list);
      sidemarker_pub.publish(sidemarker);

    }
    r.sleep();
  }

}
