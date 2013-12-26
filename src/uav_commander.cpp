/***************************************************************************
* Copyright (C) 2013 - 2014 by                                             *
* Tarek Taha, Khalifa University Robotics Institute KURI                   *
*                     <tarek.taha@kustar.ac.ae>                            *
*                                                                          *
*                                                                          *
* This program is free software; you can redistribute it and/or modify     *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation; either version 2 of the License, or        *
* (at your option) any later version.                                      *
*                                                                          *
* This program is distributed in the hope that it will be useful,          *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU General Public License for more details.                             *
*                                                                          *
* You should have received a copy of the GNU General Public License        *
* along with this program; if not, write to the                            *
* Free Software Foundation, Inc.,                                          *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              *
***************************************************************************/
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
