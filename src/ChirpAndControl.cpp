/*********************************************************************************************
* Copyright (C) 2013 - 2014 by                                                               *
* Bara Emran, Rui P. de Figueiredo and Tarek Taha Khalifa University Robotics Institute KURI *
* <bara.emran@kustar.ac.ae>, <rui.defigueiredo@kustar.ac.ae> <tarek.taha@kustar.ac.ae>       *                     	  *
*                                                                          	                 *
*                                                                           	             *
* This program is free software; you can redistribute it and/or modify     	                 *
* it under the terms of the GNU General Public License as published by     	                 *
* the Free Software Foundation; either version 2 of the License, or        	                 *
* (at your option) any later version.                                      	                 *
*                                                                          	                 *
* This program is distributed in the hope that it will be useful,          	                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           	                 *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             	                 *
* GNU General Public License for more details.                              	             *
*                                                                          	                 *
* You should have received a copy of the GNU General Public License        	                 *
* along with this program; if not, write to the                            	                 *
* Free Software Foundation, Inc.,                                          	                 *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              	                 *
**********************************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int32.h"

#include "visualeyez_tracker/TrackerPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "uav_commander/ControlInfo.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <uav_commander/PIDControlConfig.h>
//#include "pctx_control/Control.h"

#include "uav_commander/AllTopic.h"

#include <sstream>
#include <sys/time.h>

#define PI 3.14159265
#define BILLION 1000000000

double x,y,z;
const double epsilon = 0.1;

double Sat (double num, double Max , double Min);

inline bool equalFloat(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}

class PositionCommand
{
public:

    int     count;
    bool    init_;
    bool    control_;
    bool    got_pose_update_;
    bool    got_goal_;
    double  period_;
    bool    load_gains_;
    bool    chirp_enable_;

    Eigen::Matrix<double,6,1> Kp;
    Eigen::Matrix<double,6,1> Kd;
    Eigen::Matrix<double,6,1> Ki;
    Eigen::Matrix<double,6,1> goal_pose_;
    Eigen::Matrix<double,6,1> current_pose_;
    Eigen::Matrix<double,6,1> previous_pose_;
    Eigen::Matrix<double,6,1> accum_error_;
    Eigen::Matrix<double,6,1> previous_error_;
    Eigen::Matrix<double,3,3> rot_matrix_;

    ros::NodeHandle n_;
    ros::Subscriber goal_sub;
    ros::Subscriber pose_sub;
    ros::Publisher  vel_cmd;
    ros::Publisher  control_info_pub;
    ros::Publisher  current_error_pub ;
    ros::Time       previous_time_;
    ros::Time       time0_chirp_;
    geometry_msgs::PoseStamped Origin_;
    uav_commander::ControlInfo control_info_msg;
    ros::Subscriber chirp_enable_sub;

    dynamic_reconfigure::Server<uav_commander::PIDControlConfig> server;
    dynamic_reconfigure::Server<uav_commander::PIDControlConfig>::CallbackType dynamic_function;

    PositionCommand(ros::NodeHandle & n);
    void control();
    double chirp_command();

private:
    void getGoal   (const  geometry_msgs::PoseStamped::ConstPtr    & goal);
    void getPose   (const  geometry_msgs::PoseStamped::ConstPtr    & Pose);
    void dynamic   (       uav_commander::PIDControlConfig         &config, uint32_t level);
    void chirp_enable_func(const std_msgs::Int32::ConstPtr & Enable);

};

PositionCommand::PositionCommand(ros::NodeHandle & n) :
    n_(n), control_(false), init_(true)
{
    ROS_INFO("initialization");

    pose_sub            = n_.subscribe("/uav/pose" , 1, &PositionCommand::getPose   , this);
    goal_sub            = n_.subscribe("/uav/goal" , 1, &PositionCommand::getGoal   , this);
    vel_cmd             = n_.advertise <geometry_msgs::Twist       > ("/cmd_vel"      , 1);
    control_info_pub    = n_.advertise <uav_commander::ControlInfo > ("/control_info" , 1);
    current_error_pub   = n_.advertise <geometry_msgs::Pose        > ("/current_error", 1);
    chirp_enable_sub    = n_.subscribe("/chirp_enable" , 1, &PositionCommand::chirp_enable_func, this);

    Kp << 0 , 0 , 0 , 0 , 0 , 0;
    Ki << 0 , 0 , 0 , 0 , 0 , 0;
    Kd << 0 , 0 , 0 , 0 , 0 , 0;

    previous_error_  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    accum_error_     << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    dynamic_function = boost::bind(&PositionCommand::dynamic,this, _1, _2);
    server.setCallback(dynamic_function);
    return;
}


void PositionCommand::control()
{
    geometry_msgs::Twist        vel_cmd_msg;
    geometry_msgs::Pose         current_error_msg ;

    Eigen::Matrix<double,6,1>   current_error;

    if (control_)
    {
        current_error= goal_pose_ - current_pose_;

        // if the error is less than epsilon goal is reached
        if(current_error.norm() < .1)
        {
            ROS_INFO("Reached goal!");
        }

        accum_error_ = accum_error_+ period_*(current_error + previous_error_)/2.0;

        Eigen::Matrix<double,6,6> Cp = Kp * current_error.transpose();
        Eigen::Matrix<double,6,6> Ci = Ki * accum_error_.transpose();
        Eigen::Matrix<double,6,6> Cd = Kd * (current_error - previous_error_).transpose()/period_;

        //std::cout << "Cp ="  <<  Cp << std::endl;
        //std::cout << "Ci ="  <<  Ci << std::endl;
        //std::cout << "Cd ="  <<  Cd << std::endl;

        Eigen::Matrix<double,6,6> vel_cmd_current = Cp + Ci + Cd ;
        Eigen::Matrix<double,3,1> linear_vel_cmd_l_frame;

        linear_vel_cmd_l_frame << vel_cmd_current(0,0),
                vel_cmd_current(1,1),
                vel_cmd_current(2,2);

        Eigen::Matrix<double,3,1> linear_vel_cmd_g_frame = rot_matrix_.inverse() * linear_vel_cmd_l_frame;

        if (chirp_enable_)
        {
            linear_vel_cmd_g_frame(0,0) = linear_vel_cmd_g_frame(0,0) + chirp_command();
        }

        vel_cmd_msg.linear.x  = Sat(linear_vel_cmd_g_frame(0,0),1,-1);
        vel_cmd_msg.linear.y  = Sat(linear_vel_cmd_g_frame(1,0),1,-1);
        vel_cmd_msg.linear.z  = Sat(linear_vel_cmd_g_frame(2,0),1,-1);
        vel_cmd_msg.angular.z = Sat(vel_cmd_current(5,5),1,-1);

        previous_error_     = current_error;
        control_            = false;

        current_error_pub.publish(current_error_msg);

        std::cout << "CE: x ="  <<  current_error(0,0)
                  << " y = "    <<  current_error(1,0)
                  << " z = "    <<  current_error(2,0)
                  << " w = "    <<  current_error(5,0)
                  << std::endl;
    }

    current_error_msg.position.x    = current_error(0,0);
    current_error_msg.position.y    = current_error(1,0);
    current_error_msg.position.z    = current_error(2,0);
    current_error_msg.orientation.x = current_error(0,0);
    current_error_msg.orientation.y = current_error(1,0);
    current_error_msg.orientation.z = current_error(2,0);

    vel_cmd.publish(vel_cmd_msg);

    //    std::cout << "AE: x ="  <<  accum_error_(0,0)
    //              << " y = "    <<  accum_error_(1,0)
    //              << " z = "    <<  accum_error_(2,0)
    //             << " w = "    <<  accum_error_(5,0)
    //              << std::endl;

    std::cout << "Kp = "  << Kp.transpose()<< std::endl;
    std::cout << "Ki = "  << Ki.transpose()<< std::endl;
    std::cout << "Kd = "  << Kd.transpose()<< std::endl;

    std::cout << "vel: x =" <<  vel_cmd_msg.linear.x
              << " y = "    <<  vel_cmd_msg.linear.y
              << " z = "    <<  vel_cmd_msg.linear.z
              << " w = "    <<  vel_cmd_msg.angular.z
              << std::endl;

}
void PositionCommand::chirp_enable_func(const std_msgs::Int32::ConstPtr & Enable)
{
    if (Enable->data == 1)
    {
        chirp_enable_ = true;
        time0_chirp_ = ros::Time::now();
    }
    else
    {
        chirp_enable_ = false;
    }
}
void PositionCommand::getPose(const geometry_msgs::PoseStamped::ConstPtr & Pose)
{
    rot_matrix_= Eigen::Quaterniond(Pose->pose.orientation.w,
                                    Pose->pose.orientation.x,
                                    Pose->pose.orientation.y,
                                    Pose->pose.orientation.z).matrix();

    Eigen::Matrix<double,3,1> euler = rot_matrix_.eulerAngles(2, 1, 0);

    double yaw   = euler(0,0);
    double pitch = euler(1,0);
    double roll  = euler(2,0);

    current_pose_ <<    Pose->pose.position.x,
            Pose->pose.position.y,
            Pose->pose.position.z,
            roll,
            pitch,
            yaw;

    // if it is the first time: initialize the function
    if(init_)
    {
        ROS_INFO("initialize");
        previous_time_  = ros::Time::now();
        goal_pose_      << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        init_           = false;
        control_        = false;
        chirp_enable_   = false;
    }

    ros::Time current_time  = Pose->header.stamp;
    period_                 = current_time.toSec() - previous_time_.toSec();
    previous_time_          = current_time;

    control_ = true;

    control_info_msg.Period         = period_;
    control_info_msg.header.stamp   = current_time;
    control_info_pub.publish(control_info_msg);


    //    std::cout << "CP: x = " <<  current_pose_(0,0)
    //              << " y = "    <<  current_pose_(1,0)
    //              << " z = "    <<  current_pose_(2,0)
    //              << " w = "    <<  current_pose_(5,0)
    //              << std::endl;
    //    std::cout << "Period ="  <<  period_ << std::endl;
}
void PositionCommand::getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal)
{

    Eigen::Matrix<double,3,1> euler = Eigen::Quaterniond(goal->pose.orientation.w,
                                                         goal->pose.orientation.x,
                                                         goal->pose.orientation.y,
                                                         goal->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw   = euler(0,0);
    double pitch = euler(1,0);
    double roll  = euler(2,0);

    goal_pose_ <<   goal->pose.position.x,
            goal->pose.position.y,
            goal->pose.position.z,
            roll,
            pitch,
            yaw;

    ROS_INFO("GOT NEW GOAL");
    std::cout << "goal_pose: " << goal_pose_.transpose()<< std::endl;

}
double PositionCommand::chirp_command ()
{
    double max_freq = 5.0;
    double min_freq = 0.0;
    double max_amp  = 0.3;
    double min_amp  = 0.05;
    double duration = 15.0;
    double Freq, Amp, time, velocity ,f_slope , a_slope;
    ros ::Time curr_time = ros::Time::now();

    f_slope = (max_freq - min_freq  ) / duration;
    a_slope = (max_amp  - min_amp   ) / duration;


    time = curr_time.toSec() - time0_chirp_.toSec();

    Amp  = time * a_slope + min_amp;
    Freq = time * f_slope + min_freq;

    velocity = -Amp * sin ( 2 * PI * Freq * time);

    std::cout   << "f = "           << Freq
                << "\ttime = "       << time
                << "\tvelocity = "   << velocity
                << "\tAmp = "        << Amp
                << std::endl;

    if (Freq >= max_freq)
    {
        chirp_enable_ = false;
        velocity = 0;
    }
    return velocity;
}

double Sat (double num, double Max , double Min){
    if (num > Max)      {num = Max;}
    else if (num < Min) {num = Min;}
    return num;
}
void PositionCommand::dynamic(uav_commander::PIDControlConfig &config, uint32_t level)
{
    if (config.PID_Control)
    {
        ros::NodeHandle n_priv("~");
        n_priv.param<double>("kp_x" , config.Kp_x , 0.71);
        n_priv.param<double>("kp_y" , config.Kp_y , 0.72);
        n_priv.param<double>("kp_z" , config.Kp_z , 0.73);
        n_priv.param<double>("kp_w" , config.Kp_w , 0.74);
        n_priv.param<double>("ki_x" , config.Ki_x , 0.71);
        n_priv.param<double>("ki_y" , config.Ki_y , 0.72);
        n_priv.param<double>("ki_z" , config.Ki_z , 0.73);
        n_priv.param<double>("ki_w" , config.Ki_w , 0.74);
        n_priv.param<double>("kd_x" , config.Kd_x , 0.71);
        n_priv.param<double>("kd_y" , config.Kd_y , 0.72);
        n_priv.param<double>("kd_z" , config.Kd_z , 0.73);
        n_priv.param<double>("kd_w" , config.Kd_w , 0.74);
    }
    config.PID_Control = 0;
    Kp << config.Kp_x , config.Kp_y , config.Kp_z , 0 , 0 , config.Kp_w;
    Ki << config.Ki_x , config.Ki_y , config.Ki_z , 0 , 0 , config.Ki_w;
    Kd << config.Kd_x , config.Kd_y , config.Kd_z , 0 , 0 , config.Kd_w;

    control_info_msg.Kp.x = config.Kp_x ;
    control_info_msg.Kp.y = config.Kp_y ;
    control_info_msg.Kp.z = config.Kp_z ;
    control_info_msg.Kp.r = 0 ;
    control_info_msg.Kp.p = 0 ;
    control_info_msg.Kp.w = config.Kp_w ;

    control_info_msg.Ki.x = config.Ki_x ;
    control_info_msg.Ki.y = config.Ki_y ;
    control_info_msg.Ki.z = config.Ki_z ;
    control_info_msg.Ki.r = 0 ;
    control_info_msg.Ki.p = 0 ;
    control_info_msg.Ki.w = config.Ki_w ;

    control_info_msg.Kd.x = config.Kd_x ;
    control_info_msg.Kd.y = config.Kd_y ;
    control_info_msg.Kd.z = config.Kd_z ;
    control_info_msg.Kd.r = 0 ;
    control_info_msg.Kd.p = 0 ;
    control_info_msg.Kd.w = config.Kd_w ;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "chirp_controler");
    ros::NodeHandle n;

    ros::Rate       loop_rate(30);
    PositionCommand position_commander(n);

    while (ros::ok())
    {
        position_commander.control();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}


