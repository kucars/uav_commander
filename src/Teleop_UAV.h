#ifndef TELEOP_UAV_H
#define TELEOP_UAV_H

#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include <ardrone_autonomy/Navdata.h>

#define Arrow_R 0x43 // Right Arrow
#define Arrow_L 0x44 // Left Arrow
#define Arrow_U 0x41 // Up Arrow
#define Arrow_D 0x42 // Down Arrow
#define KEY_w   'w'
#define KEY_s   's'
#define KEY_d   'd'
#define KEY_a   'a'
#define KEY_f   'f'
#define KEY_l   'l'
#define KEY_e   'e'
#define KEY_t   't'
#define KEY_q   'q'
#define KEY_EMR ' '

class Teleop_UAV
{
public:
    Teleop_UAV(ros::NodeHandle & n,ros::NodeHandle & n_priv);
    void quit();
    void land();
    void keyLoop();
    bool keyread();
    void watchdog();
    void take_off();
    void emergency();
    void flat_trim();
    void check_hovering();
    void move (int x, int y, int z, int w);
    void get_navigation(const ardrone_autonomy::Navdata::ConstPtr & Nav);

private:
    char c_;
    int kfd_;
    struct termios cooked_, raw_;
    bool hovering_,watch_dog_;
    double x_scale_,y_scale_,w_scale_,z_scale_,emergency_battery_level_,take_off_time_;
    ardrone_autonomy::Navdata navigation_data_;
    ros::Time last_publish_;
    ros::NodeHandle n_,n_priv_;

    ros::Publisher  vel_cmd_pub,takeoff_pub,land_pub,reset_pub;
    ros::Subscriber navdata_sub_;
    ros::ServiceClient flattrim_client_;
};

#endif // TELEOP_UAV_H
