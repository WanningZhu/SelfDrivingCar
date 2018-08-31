
//Add ability for students to dynamically reconfigure the heading_rate and the velocity
//display visualization marker at cars position and heading
//should be able to run rviz and rqt_reconfigure to drive car around

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <an_vehicle/dyn_reconfigConfig.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_broadcaster.h"
#include "std_msgs/Float64.h"

class Vehicle
{
public:
  Vehicle()
  {
  }

//  double velocity = 25;
  double velocity = 25;
  double heading = 1.0;
  double heading_rate = 0;
  double steering_angle = 0;
  double acceleration = 0;
};

void callback(an_vehicle::dyn_reconfigConfig &config, uint32_t level, Vehicle* vehicle) {
  vehicle->heading_rate = config.heading_rate;
  vehicle->velocity = config.velocity;
  ROS_INFO("heading_rate = %f, velocity = %f",
           vehicle->heading_rate, vehicle->velocity);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "vehicle");

  dynamic_reconfigure::Server<an_vehicle::dyn_reconfigConfig> server;
  dynamic_reconfigure::Server<an_vehicle::dyn_reconfigConfig>::CallbackType f;

  Vehicle* vehicle = new Vehicle();
  f = boost::bind(&callback, _1, _2, vehicle);
  server.setCallback(f);

  ros::NodeHandle node;
  ros::Publisher marker_pub = node.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );
  tf::TransformBroadcaster br;
\
  double dt = 0.02;

  ros::Rate loop_rate(1/dt);

  double X=0, Y=0;
  geometry_msgs::Quaternion odom_quat;

  while(ros::ok()) {
    ros::spinOnce();

    X += vehicle->velocity * cos(vehicle->heading) * dt;
    Y += vehicle->velocity * sin(vehicle->heading) * dt;

    vehicle->heading = vehicle->heading + vehicle->heading_rate * dt;
    odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0, 0, vehicle->heading);

    ROS_INFO("X = %f, Y = %f, heading = %f", X, Y, vehicle->heading);
    ROS_INFO("velocity = %f, heading_rate = %f", vehicle->velocity, vehicle->heading_rate);

    //publish visualization marker here
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "vehicle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = X;
    marker.pose.position.y = Y;
    marker.pose.position.z = 0.75;
    marker.pose.orientation = odom_quat;

    marker.scale.x = 5.0;
    marker.scale.y = 2.0;
    marker.scale.z = 1.5;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    marker.lifetime = ros::Duration();

    sleep(1);

    marker_pub.publish( marker );

    loop_rate.sleep();
  }
}
