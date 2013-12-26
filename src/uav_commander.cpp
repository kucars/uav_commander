#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pctx_control/Control.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::Publisher uav_commands = n.advertise<pctx_control::Control>("sendPCTXControl", 1000);
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok())
    {
        pctx_control::Control controlMessage;
        controlMessage.channel = 2;
        controlMessage.value   = count%1020;
        ROS_INFO("UAV_Commander sending to channel:%d value:%d",controlMessage.channel,controlMessage.value);
        uav_commands.publish(controlMessage);
        ros::spinOnce();
        loop_rate.sleep();
        count+=20;
    }
    return 0;
}
