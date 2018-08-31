#include "rotation_test/rotation_test_node.h"


int main(int argc, char **argv) {
  ros::init(argc, argv, "rotation_test_node");
  ros::NodeHandle node;
  
  tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  
  double x, y, z;
  double roll, pitch, yaw;
  double qx, qy, qz, qw;
  
  ros::Rate loop_rate(10);
  int loop_count =0;
  
  while(ros::ok()) {
    loop_count++;
    if (loop_count > 100) { loop_count = 1; }
    
    x = 0;//5;//25-fabs(50-loop_count);
    y = 0;
    z = 0;
    transform.setOrigin(tf::Vector3(x, y, z));
    
    roll = 0;
    pitch = 30;
    yaw = 0;
   // q.setRPY(roll, pitch, yaw);  // comment this out if not using roll pitch and yaw to set rotation
        
    q[0] = 0.099033;          //x
    q[1] = 0.369597;          //y
    q[2] = 0.239116;          //z
    q[3] = 0.892397 * (100/(double)loop_count);                 //w
    q.normalize();
       
    transform.setRotation(q);
    
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ego", "ego_transform"));
    loop_rate.sleep();
  }
  
}
