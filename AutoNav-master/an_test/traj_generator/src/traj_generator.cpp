#include "ros/ros.h"
#include <vector>
#include <cstdio>
#include "an_messages/trajectory.h"
#include "an_messages/traj_pt.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf/transform_broadcaster.h>



int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_generator");
  ros::start();
  ros::NodeHandle nh;

  ros::Publisher traj_pub = nh.advertise<an_messages::trajectory>("planner_trajectory", 1000, true);

  an_messages::trajectory traj;
  traj.traj.resize(20000);
  ros::Time start = ros::Time::now();
  traj.header.frame_id = "/map";

  for (int tidx=0; tidx < traj.traj.size(); tidx++) {
    traj.traj[tidx].header.stamp = start + ros::Duration(0.1/25);
    traj.traj[tidx].header.frame_id = "/map";
    traj.traj[tidx].position.x = (double)tidx/10;
    traj.traj[tidx].position.y = 0;
    traj.traj[tidx].position.theta = 0;
    traj.traj[tidx].velocity.linear.x = 25;
    traj.traj[tidx].velocity.linear.y = 0;
    traj.traj[tidx].velocity.linear.z = 0;

    if (tidx < 5) {
      ROS_WARN("x:%f y:%f th:%f vel:%f", traj.traj[tidx].position.x, traj.traj[tidx].position.y, traj.traj[tidx].position.theta, traj.traj[tidx].velocity.linear.x);
    }
  }

  traj_pub.publish(traj);

  ros::Rate r(0.1);
  while(ros::ok() ) {
    r.sleep();
  }
}