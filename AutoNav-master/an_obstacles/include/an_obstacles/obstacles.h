/* Copyright (C) 2018 RobotWits, LLC - All Rights Reserved
 * Unauthorized copying of this file, via any medium is strictly prohibited
 * Written by Jonathan Butzke <instructor@robotwits.com>, January, 2018
 */

#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "an_messages/trajectory.h"
#include "an_messages/traj_pt.h"
#include "an_messages/obstacle.h"
#include "an_messages/obstacles.h"
#include <vector>
#include <string>


#define TIME_DELAY 5.0
