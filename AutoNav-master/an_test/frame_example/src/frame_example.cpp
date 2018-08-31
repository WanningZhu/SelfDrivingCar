#include "frame_example/frame_example.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rotation_test_node");
  ros::NodeHandle node;
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  double roll, pitch, yaw;
  visualization_msgs::Marker marker_;

  ros::Publisher marker_pub_ = node.advertise<visualization_msgs::Marker>("visualization_marker", 5, false);

  marker_.header.frame_id = "ego";
  marker_.header.seq = 0;
  marker_.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker_.mesh_use_embedded_materials = true;
  marker_.mesh_resource = "package://an_scenario/meshes/Generic_Coupe_v02.dae";
  marker_.ns = "ego";
  marker_.id = 1;
  marker_.scale.x = 0.0030;
  marker_.scale.y = 0.0030;
  marker_.scale.z = 0.0030;
  marker_.action = visualization_msgs::Marker::ADD;
  marker_.lifetime = ros::Duration();
  marker_.frame_locked = true;
  marker_.color.r = 0;
  marker_.color.g = 0;
  marker_.color.b = 0;
  marker_.color.a = 0;
  marker_.pose.position.x = 0;
  marker_.pose.position.y = 0;
  marker_.pose.position.z = 0;
  marker_.pose.orientation.x = 0;
  marker_.pose.orientation.y = 0;
  marker_.pose.orientation.z = 0;
  marker_.pose.orientation.w = 0;

  ros::Rate loop_rate(10);

  roll = 0;
  pitch = 0;
  yaw = 45;
  int count=0, d=0;
  double X=0, Y=0, Z=0;

  while(ros::ok()) {

    if (count < 20) {
      pitch++;
    } else if (count < 60) {
      pitch--;
    } else if (count < 100) {
      pitch++;
    } else if (count < 140) {
      pitch--;
    } else if (count < 200) {
      pitch = 0;
      yaw++;
    } else if (count < 220) {
      yaw--;
    } else if (count < 260) {
      yaw++;
    } else {
      yaw--;
    }

    //Not real equations of motion!!! don't use these!!
    X += cos((yaw+pitch)*6.28/360)/10;
    Y += sin(yaw*6.28/360)/10;
    Z += sin(pitch*6.28/360)/10;

    count++;
    transform.setOrigin(tf::Vector3(X,Y,Z));
    d++;

    // send map-aligned frame

    q.setRPY(0, 0, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "map_aligned"));

    q.setRPY(0, 0, yaw*6.28/360);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "stabilized"));

    q.setRPY(roll*6.28/360, pitch*6.28/360, yaw*6.28/360);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "ego"));

    marker_pub_.publish(marker_);

    loop_rate.sleep();
  }

}
