#include "ardrone_interface.h"

ArdroneInterface::ArdroneInterface():
    ph_("~"),
    hovering_(false),
    emergency_(false)
{
    ROS_INFO_STREAM ("Initialize Ardrone Interface");

    vel_pub_            = ph_.advertise      <geometry_msgs::Twist>     ("/cmd_vel"    		, 1	);
    take_off_pub_       = ph_.advertise      <std_msgs::Empty >      	("/ardrone/takeoff"	, 1	);
    land_pub_           = ph_.advertise      <std_msgs::Empty >      	("/ardrone/land"   	, 1	);
    reset_pub_          = ph_.advertise      <std_msgs::Empty >      	("/ardrone/reset"  	, 1	);
    flat_trim_client_   = ph_.serviceClient  <std_srvs::Empty>       	("/ardrone/flattrim"    );
    nav_data_sub_       = ph_.subscribe      ("/ardrone/navdata"   , 1, &ArdroneInterface::readNav    , this);

    std::cout<< "Read Parameters" << std::endl;

    ph_.param<double>("battery_level_percent"	,emergency_battery_level_	, 30.0);
    ph_.param<double>("take_off_time_sec"       ,take_off_time_             , 10.0);
    ph_.param<double>("max_vel_"                ,max_vel_                   ,  1.0 );
    ph_.param<double>("min_vel_"                 ,min_vel_                  ,- 1.0 );

    std::cout<< "battery_level_percent = "  << emergency_battery_level_ << std::endl;
    std::cout<< "take_off_time_sec  = "     << take_off_time_           << std::endl;
    std::cout<< "max_vel_ = "               << max_vel_                 << std::endl;
    std::cout<< "min_vel_  = "              << min_vel_                 << std::endl;
}   

void ArdroneInterface::getInfo( double &emr_bat_level, double &takeOff_time) const
{
    emr_bat_level   = emergency_battery_level_;
    takeOff_time    = take_off_time_;
}
void ArdroneInterface::setInfo( double  emr_bat_level, double  takeOff_time, bool hover)
{
    emergency_battery_level_    = emr_bat_level;
    take_off_time_              = takeOff_time;
    hovering_                   = hover;
}

void ArdroneInterface::takeOff ()
{
    flatTrim ();
    std::cout << "Taking Off" << std::endl;
    std_msgs::Empty msg;
    take_off_pub_.publish(msg);
    hoveringTimeOut();
}

void ArdroneInterface::emergency()
{
    std::cout << "Toggle Emergency" << std::endl;
    std_msgs::Empty msg;
    reset_pub_.publish(msg);
}

void ArdroneInterface::land ()
{
    std::cout << "Landding" << std::endl;
    hovering_ = false;
    std_msgs::Empty msg;
    land_pub_.publish(msg);
    //while (nav_data_.state != state(Landed ));
}

void ArdroneInterface::flatTrim ()
{
    std::cout << "Flat Trim" << std::endl;
    std_srvs::Empty srv;
    flat_trim_client_.call(srv);
}

void ArdroneInterface::hoveringTimeOut(){
    ros::Time time0 = ros::Time::now();
    while (!checkHovering())
    {
        if(ros::Time::now() > time0 + ros::Duration(take_off_time_))
        {
            land ();
            break;
        }
    }
}

bool ArdroneInterface::checkHovering()
{
    if(nav_data_.state == state(Hovering))
    {
        std::cout << "Ardrone is Hovering" << std::endl;
        hovering_ = true;
        return 1;
    }
    else
        return 0;
}

bool ArdroneInterface::velCom (geometry_msgs::Twist & vel_msg)
{
    if (hovering_)
    {
        saturation (vel_msg.linear.x    , max_vel_ , min_vel_);
        saturation (vel_msg.linear.y    , max_vel_ , min_vel_);
        saturation (vel_msg.linear.z    , max_vel_ , min_vel_);
        saturation (vel_msg.angular.z   , max_vel_ , min_vel_);
        vel_pub_.publish(vel_msg);
        return 1;
    }
    else
        return 0;
}

void ArdroneInterface::readNav(const ardrone_autonomy::Navdata::ConstPtr & Nav)
{
    nav_data_ = *Nav;
    if (nav_data_.batteryPercent < emergency_battery_level_ && !emergency_)
    {
        emergency_ = true;
        std::cout << "Emergency Landing" << std::endl;
        land();
    }
}

void ArdroneInterface::getNav(ardrone_autonomy::Navdata & Nav) const
{
    Nav = nav_data_;
}

void ArdroneInterface::saturation (double &num, double Max , double Min){
    if (num > Max)      {num = Max;}
    else if (num < Min) {num = Min;}
    return;
}
