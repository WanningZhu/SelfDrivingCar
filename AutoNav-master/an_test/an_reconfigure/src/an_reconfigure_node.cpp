#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <an_reconfigure/dyn_reconfigConfig.h>

void callback(an_vehicle::dyn_reconfigConfig &config, uint32_t level)
{
    ROS_INFO("heading_rate: %f, other_rate: %f",
             config.heading_rate, config.other_rate);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "an_reconfigure_node");

    dynamic_reconfigure::Server<an_vehicle::dyn_reconfigConfig> server;
    dynamic_reconfigure::Server<an_vehicle::dyn_reconfigConfig>::CallbackType f;
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);
    ros::spin();
    return 0;
}
