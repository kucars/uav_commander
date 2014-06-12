#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sys/time.h"
#define PI 3.14159265

int main(int argc, char **argv)
{
    double max_freq = 5.0;
    double min_freq = 0.0;
    double max_amp  = 0.3;
    double min_amp  = 0.05;
    double duration = 15.0;
    double Freq, Amp, time, velocity ,f_slope , a_slope;

    geometry_msgs   ::Twist cmd_vel_msg;
    ros             ::Time time0 , curr_time;

    ros::init(argc, argv, "chirp_commander");
    ros::NodeHandle h;
    ros::Publisher chirp_pub = h.advertise <geometry_msgs::Twist> ("/cmd_vel",1);
    ros::Rate loop_rate(40);

    time0 = ros::Time::now();

    f_slope = (max_freq - min_freq  ) / duration;
    a_slope = (max_amp  - min_amp   ) / duration;

    while (ros::ok())
    {
        curr_time = ros::Time::now();
        time = curr_time.toSec() - time0.toSec();

        Amp  = time * a_slope + min_amp;
        Freq = time * f_slope + min_freq;

        velocity = -Amp * sin ( 2 * PI * Freq * time);

        cmd_vel_msg.linear.x = velocity;
        chirp_pub.publish(cmd_vel_msg);

        std::cout   << "f = "           << Freq
                    << "\ttime = "       << time
                    << "\tvelocity = "   << velocity
                    << "\tAmp = "        << Amp
                    << std::endl;

        if (Freq >= max_freq)
            break;

        ros::spinOnce();
        loop_rate.sleep();
    }
    cmd_vel_msg.linear.x = 0;
    cmd_vel_msg.linear.y = 0;
    chirp_pub.publish(cmd_vel_msg);

    return 0;
}
