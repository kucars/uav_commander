#ifndef TELEOP_UAV_H
#define TELEOP_UAV_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include "ardrone_interface.h"

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
#define KEY_EMR ' '

int kfd = 0;
struct termios cooked, raw;

class TeleopUAV
{
public:
    TeleopUAV();
    void keyLoop();
    void watchdog();
private:
    ArdroneInterface    interface_;
    ros::NodeHandle     ph_;
    ros::Time           first_publish_;
    ros::Time           last_publish_;
    boost::mutex        publish_mutex_;

    double  l_scale_;
    double  a_scale_;

    void sendVel(geometry_msgs::Twist & vel);
};

#endif // TELEOP_UAV_H
