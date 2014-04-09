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
//#include "pctx_control/Control.h"
#include "visualeyez_tracker/TrackerPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <Eigen/Eigen>
#include <std_msgs/String.h>
#include <dynamic_reconfigure/server.h>
#include <uav_commander/PIDControlConfig.h>

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

    ros::Time previous_time;
    int count;
    bool init_;
    bool control_;
    bool got_pose_update_;


    double freq_;
    double period_;


    // PID gains

    Eigen::Matrix<double,6,1> Kp;
    Eigen::Matrix<double,6,1> Kd;
    Eigen::Matrix<double,6,1> Ki;
    Eigen::Matrix<double,6,1> previous_error_;
    Eigen::Matrix<double,6,1> accum_error_;
    Eigen::Matrix<double,6,6> vel_cmd_previous;
    Eigen::Matrix<double,6,1> previous_pose_;
    Eigen::Matrix<double,6,1> current_pose_;
    Eigen::Matrix<double,6,1> goal_pose_;

    ros::NodeHandle n_;
    ros::Subscriber goal_sub;
    ros::Subscriber pose_sub;
    ros::Publisher  vel_cmd;

    dynamic_reconfigure::Server<uav_commander::PIDControlConfig> server;
    dynamic_reconfigure::Server<uav_commander::PIDControlConfig>::CallbackType dynamic_function;



    PositionCommand(ros::NodeHandle & n, double & freq);
    void control();

private:
    void getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal);
    void getUpdatedPose(const geometry_msgs::PoseStamped::ConstPtr & Pose);
    void dynamic_callback(uav_commander::PIDControlConfig &config, uint32_t level);

};

PositionCommand::PositionCommand(ros::NodeHandle & n, double & freq) :
    n_(n),     freq_(freq), control_(true), init_(true)
{
    ROS_INFO("initialization");

    ROS_INFO_STREAM("freq: "<< freq_);
    pose_sub = n_.subscribe("/uav/pose" , 1, &PositionCommand::getUpdatedPose, this);
    goal_sub = n_.subscribe("/uav/goal" , 1, &PositionCommand::getGoal       , this);
    vel_cmd  = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    previous_error_  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    accum_error_     << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Kp << 1 , 1 , 1 , 0 , 0 , 1;
    Ki << 0 , 0 , 0 , 0 , 0 , 0;
    Kd << 0 , 0 , 0 , 0 , 0 , 0;

    dynamic_function = boost::bind(&PositionCommand::dynamic_callback,this, _1, _2);
    server.setCallback(dynamic_function);
    return;
}


void PositionCommand::control()
{
    geometry_msgs::Twist vel_cmd_msg;

    //period_ = 0.05;

    Eigen::Matrix<double,6,1> current_error = goal_pose_ - current_pose_;
    //Eigen::Matrix<double,6,1> pose_error    = previous_pose_ - current_pose_;
    //ROS_INFO("Current_Error");
    std::cout << "CE  :x= " <<  current_error(0,0)
              << " y = "    <<  current_error(1,0)
              << " z = "    <<  current_error(2,0)
              << " w = "    <<  current_error(5,0)
              << std::endl;
    // if the error is less than epsilon goal is reached
    if(current_error.norm() < epsilon) // CHANGE THIS
    {
        ROS_INFO("Reached goal!");

    }
    if (control_)
    {
        Eigen::Matrix<double,6,1> accum_error_ = accum_error_+ period_*(current_error + previous_error_)/2;

        Eigen::Matrix<double,6,6> Cp = Kp * current_error.transpose();
        Eigen::Matrix<double,6,6> Ci = Ki * accum_error_.transpose();
        Eigen::Matrix<double,6,6> Cd = Kd * (current_error - previous_error_).transpose()/period_;

        Eigen::Matrix<double,6,6> vel_cmd_current = Cp + Ci;

        vel_cmd_msg.linear.x  = Sat(vel_cmd_current(0,0),1,-1);
        vel_cmd_msg.linear.y  = Sat(vel_cmd_current(1,1),1,-1);
        vel_cmd_msg.linear.z  = Sat(vel_cmd_current(2,2),1,-1);
        vel_cmd_msg.angular.z = Sat(vel_cmd_current(5,5),1,-1);


        previous_error_ = current_error;

        std::cout << "vel :x= " <<  vel_cmd_msg.linear.x
                  << " y = "    <<  vel_cmd_msg.linear.y
                  << " z = "    <<  vel_cmd_msg.linear.z
                  << " w = "    <<  vel_cmd_msg.angular.z
                  << std::endl;

        //std::cout << "accum_error_" << accum_error_.transpose() <<std::endl;
    }


    vel_cmd.publish(vel_cmd_msg);
   // control_=false;

}

void PositionCommand::getUpdatedPose(const geometry_msgs::PoseStamped::ConstPtr & Pose)
{
   // control_=true;
    //ROS_INFO("GOT POSITION");
    Eigen::Matrix<double,3,1> euler=  Eigen::Quaterniond(Pose->pose.orientation.w,
                                                         Pose->pose.orientation.x,
                                                         Pose->pose.orientation.y,
                                                         Pose->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw   = euler(0,0);
    double pitch = euler(1,0);
    double roll  = euler(2,0);

    current_pose_ << Pose->pose.position.x,
            Pose->pose.position.y,
            Pose->pose.position.z,
            roll,
            pitch,
            yaw;


    // if it the first time: initialize the function
    if(init_)
    {
        ROS_INFO("initialize");
        previous_time = ros::Time::now();
        previous_pose_ = current_pose_;
        goal_pose_ = current_pose_;
        init_ = false;
    }

    ros::Time current_time = Pose->header.stamp;
    period_ = current_time.toSec()-previous_time.toSec();
    previous_time = current_time;

    std::cout << "CP  :x= " <<  current_pose_(0,0)
              << " y = "    <<  current_pose_(1,0)
              << " z = "    <<  current_pose_(2,0)
              << " w = "    <<  current_pose_(5,0)
              << std::endl;

   // std::cout << "current_time:"<< current_time << std::endl;
   // std::cout << "period:"<<period_ << std::endl;
   // std::cout << "previous_time:"<<previous_time << std::endl;
}

void PositionCommand::getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal)
{

    Eigen::Matrix<double,6,1> old_goal_pose_;

    Eigen::Matrix<double,3,1> euler=  Eigen::Quaterniond(goal->pose.orientation.w,
                                                         goal->pose.orientation.x,
                                                         goal->pose.orientation.y,
                                                         goal->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw   = euler(0,0);
    double pitch = euler(1,0);
    double roll  = euler(2,0);



    goal_pose_ << goal->pose.position.x,
            goal->pose.position.y,
            goal->pose.position.z,
            roll,
            pitch,
            yaw;
    ROS_INFO("GOT NEW GOAL");

  //  if (old_goal_pose_ != goal_pose_)
  //  {
  //
  //      old_goal_pose_ = goal_pose_;
  //  }
    std::cout << "goal_pose: " << goal_pose_.transpose()<< std::endl;

}

double Sat (double num, double Max , double Min){
    if (num > Max)
        num = Max;
    else if (num < Min)
        num = Min;
    return num;
}
void PositionCommand::dynamic_callback(uav_commander::PIDControlConfig &config, uint32_t level)
{
    Kp << config.Kp_x , config.Kp_y , config.Kp_z , 0 , 0 , config.Kp_w;
    Ki << config.Ki_x , config.Ki_y , config.Ki_z , 0 , 0 , config.Ki_w;
    Kd << config.Kd_x , config.Kd_y , config.Kd_z , 0 , 0 , config.Kd_w;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");

    double freq;

    n_priv.param<double>("freq", freq, 50.0);
    /*
    ///////////
    // Gains //
    ///////////

    double kp_x , kp_y , kp_z , kp_r , kp_p , kp_w;
    double ki_x , ki_y , ki_z , ki_r , ki_p , ki_w;
    double kd_x , kd_y , kd_z , kd_r , kd_p , kd_w;

    Eigen::Matrix<double,6,1> Kp;
    Eigen::Matrix<double,6,1> Kd;
    Eigen::Matrix<double,6,1> Ki;

    n_priv.param<double>("kp_x", kp_x , 1.0);
    n_priv.param<double>("kp_y", kp_y , 1.0);
    n_priv.param<double>("kp_z", kp_z , 1.0);
    n_priv.param<double>("kp_r", kp_r , 0.0);
    n_priv.param<double>("kp_p", kp_p , 0.0);
    n_priv.param<double>("kp_w", kp_y , 1.0);

    n_priv.param<double>("kd_x", kd_x , 0.0);
    n_priv.param<double>("kd_y", kd_y , 0.0);
    n_priv.param<double>("kd_z", kd_z , 0.0);
    n_priv.param<double>("kd_r", kd_r , 0.0);
    n_priv.param<double>("kd_p", kd_p , 0.0);
    n_priv.param<double>("kd_w", kd_y , 0.0);

    Kp << kp_x , kp_y , kp_z , kp_r, kp_p , kp_w;
    Ki << ki_x , ki_y , ki_z , ki_r, ki_p , ki_w;
    Kd << kd_x , kd_y , kd_z , kd_r, kd_p , kd_w;

    Kp << 1 , 1 , 1 , 0 , 0 , 1;
    Ki << 0 , 0 , 0 , 0 , 0 , 0;
    Kd << 0 , 0 , 0 , 0 , 0 , 0;
    std::cout << "Kp gains:" << Kp << std::endl;
    std::cout << "Ki gains:" << Ki << std::endl;
    std::cout << "Kd gains:" << Kd << std::endl;
*/
    PositionCommand position_commander(n,freq);

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        std::cout<<position_commander.Kp<<std::endl;
        position_commander.control();
        ros::spinOnce();
        loop_rate.sleep();

    }

    return 0;
}

