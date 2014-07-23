#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <geometry_msgs/Twist.h>
#include "boost/thread/mutex.hpp"
#include "boost/thread/thread.hpp"
#include "teleop_uav.h"

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

TeleopUAV::TeleopUAV():
    ph_("~")
{
    ROS_INFO_STREAM ("Initialize Teleoperation UAV");
    std::cout<< "Read Parameters" << std::endl;

    ph_.param("scale_angular" , a_scale_, 0.1);
    ph_.param("scale_linear"  , l_scale_, 0.1);

    std::cout<< "scale_angular = " << a_scale_ << std::endl;
    std::cout<< "scale_linear  = " << l_scale_ << std::endl;

}

void TeleopUAV::watchdog()
{
    boost::mutex::scoped_lock lock(publish_mutex_);
    if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
            (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
    {
        geometry_msgs::Twist vel_msg;
        sendVel(vel_msg);
    }
}

void TeleopUAV::sendVel(geometry_msgs::Twist & vel)
{
    vel.linear.x  = l_scale_ * vel.linear.x;
    vel.linear.y  = l_scale_ * vel.linear.y;
    vel.linear.z  = l_scale_ * vel.linear.z;
    vel.angular.z = a_scale_ * vel.angular.z;
    interface_.velCom(vel);
    return;
}

void TeleopUAV::keyLoop()
{
    char c;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move the UAV.");

    while (ros::ok())
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }
        ROS_DEBUG("value: 0x%02X\n", c);
        geometry_msgs::Twist vel_msg;
        switch(c)
        {
        case Arrow_U    :   vel_msg.linear.z    =  1.0; break;
        case Arrow_D    :   vel_msg.linear.z    = -1.0; break;
        case Arrow_L    :   vel_msg.angular.z   =  1.0; break;
        case Arrow_R    :   vel_msg.angular.z   = -1.0; break;
        case KEY_w      :   vel_msg.linear.x    =  1.0; break;
        case KEY_s      :   vel_msg.linear.x    = -1.0; break;
        case KEY_d      :   vel_msg.linear.y    =  1.0; break;
        case KEY_a      :   vel_msg.linear.y    = -1.0; break;
        case KEY_f      :   interface_.flatTrim();      break;
        case KEY_l      :   interface_.land();          break;
        case KEY_EMR    :   interface_.land();          break;
        case KEY_t      :   interface_.takeOff();       break;
        case KEY_e      :   interface_.emergency();     break;
        }

        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
            first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        sendVel(vel_msg);
    }

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtlebot_teleop");


    TeleopUAV teleop_uav;
    ros::NodeHandle n;
    signal(SIGINT,quit);

    boost::thread my_thread(boost::bind(&TeleopUAV::keyLoop, &teleop_uav));
    ros::Timer timer = n.createTimer(ros::Duration(0.1), boost::bind(&TeleopUAV::watchdog, &teleop_uav));

    ros::spin();

    my_thread.interrupt() ;
    my_thread.join() ;

    return(0);
}
