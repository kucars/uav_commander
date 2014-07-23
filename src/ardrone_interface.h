#ifndef ARDRONE_INTERFACE_H
#define ARDRONE_INTERFACE_H

#include "ros/ros.h"
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>

class ArdroneInterface
{
private:
    bool    hovering_;
    bool    emergency_;
    double  emergency_battery_level_;
    double  take_off_time_;
    double  max_vel_;
    double  min_vel_;
    enum state  { Unknown       = 0, Init           = 1,
                  Landed        = 2, Flying         = 3,
                  Hovering      = 4, Test           = 5,
                  Taking_off    = 6, Goto_Fix_Point = 7,
                  Landing       = 8, Looping        = 9};
    ardrone_autonomy::Navdata nav_data_;
    ros::NodeHandle 	ph_;
    ros::Publisher  	vel_pub_;
    ros::Publisher      take_off_pub_;
    ros::Publisher      land_pub_;
    ros::Publisher      reset_pub_;
    ros::Subscriber 	nav_data_sub_;
    ros::ServiceClient 	flat_trim_client_;

    void hoveringTimeOut();
    void readNav(const ardrone_autonomy::Navdata::ConstPtr &Nav);
    void saturation (double &num, double Max , double Min);

public:
    ArdroneInterface();
    void land();
    void takeOff();
    void emergency();
    void flatTrim();
    bool velCom (geometry_msgs::Twist & vel_msg);
    void getNav(ardrone_autonomy::Navdata & Nav) const;
    bool checkHovering();
    void getInfo( double &emr_bat_level, double &takeOff_time) const;
    void setInfo( double  emr_bat_level, double  takeOff_time , bool hover);
};

#endif // AIRDRONE_INTERFACE_H
