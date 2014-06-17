#include "position_command.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle     n;
    ros::Rate           loop_rate(40);
    Position_Command    position_command (n,ros::this_node::getName());

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

