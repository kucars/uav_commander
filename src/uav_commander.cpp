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
#include "visualeyez_tracker/TrackerPose.h"

#include <sstream>

void getUpdatedPose(const visualeyez_tracker::TrackerPose& trackerPose)
{
    ROS_INFO(" Recieved Tracker Location: [%s] [%f] [%f] [%f]",trackerPose.tracker_id.c_str(),trackerPose.pose.x ,trackerPose.pose.y ,trackerPose.pose.z );
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::Publisher uav_commands = n.advertise<pctx_control::Control>("sendPCTXControl", 1000);
    ros::Subscriber sub = n.subscribe("TrackerPosition", 1000, getUpdatedPose);
    ros::Rate loop_rate(10);
    int count = 0;
    std::vector<int16_t> controlValues(9,0);
    while (ros::ok())
    {
        pctx_control::Control controlMessage;
        controlValues[0] = controlValues[1] =controlValues[2] =controlValues[3] =controlValues[4] =controlValues[5] =controlValues[6] =controlValues[7] =controlValues[8] = count%1020;
        controlMessage.values  = controlValues;
        ROS_INFO("UAV_Commander broadcasting to all channels value:%d",controlValues[0]);
        uav_commands.publish(controlMessage);
        ros::spinOnce();
        loop_rate.sleep();
        count+=20;
    }
    return 0;
}
