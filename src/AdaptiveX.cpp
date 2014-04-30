
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
#include "ardrone_autonomy/Navdata.h"
#include <sstream>
#include <sys/time.h>
#include <math.h>
#define PI 3.14159265
#define BILLION 1000000000

const double epsilon = 0.1;

double Sat (double num, double Max , double Min);
double integration (double y, double ydot , double period );

inline bool equalFloat(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}

class PositionCommand
{
public:

    int     count;
    bool    init_, control_, got_pose_update_, got_goal_;
    double  period_;
    bool    load_gains_;
    bool    Got_IMU_;
    bool    Update_Goal_;
    bool    Adaptive_;
    double  a2b_,duration_,b_,c_;
    double  a0est_,a1est_,a2est_,previous_velocity_;


    Eigen::Matrix<double,6,1> goal_pose_, current_pose_, previous_pose_, accum_error_, previous_error_;
    Eigen::Matrix<double,3,3> rot_matrix_;
    Eigen::Matrix<double,3,1> acceleration_, velocity_;
    Eigen::Matrix<double,6,1> Kp_, Kd_, Ki_;
    Eigen::Matrix<double,3,1> x_, x_dot_,B_;
    Eigen::Matrix<double,3,3> A_;

    ros::NodeHandle n_;
    ros::Subscriber goal_sub, pose_sub, nav_sub, adaptive_enable_sub;
    ros::Publisher  vel_cmd, control_info_pub,current_error_pub ;
    ros::Time       previous_time_, time0_;

    geometry_msgs::PoseStamped Origin_;

    PositionCommand(ros::NodeHandle & n);
    void control();

private:
    void getGoal    (const  geometry_msgs::PoseStamped::ConstPtr    & goal);
    void getPose    (const  geometry_msgs::PoseStamped::ConstPtr    & Pose);
    void getIMU     (const ardrone_autonomy::Navdata::ConstPtr & Nav);
    void adaptive_enable_func(const std_msgs::Int32::ConstPtr & Enable);
};

PositionCommand::PositionCommand(ros::NodeHandle & n) :
    n_(n), control_(false), init_(true)
{
    ROS_INFO("initialization");

    pose_sub            = n_.subscribe("/uav/pose"          , 1, &PositionCommand::getPose   , this);
    goal_sub            = n_.subscribe("/uav/goal"          , 1, &PositionCommand::getGoal   , this);
    nav_sub             = n_.subscribe("/ardrone/navdata"   , 1, &PositionCommand::getIMU    , this);
    adaptive_enable_sub = n_.subscribe("/adaptive_enable" , 1, &PositionCommand::adaptive_enable_func, this);

    vel_cmd             = n_.advertise <geometry_msgs::Twist       > ("/cmd_vel"      , 1);
    current_error_pub   = n_.advertise <geometry_msgs::Pose        > ("/current_error", 1);

    previous_error_  << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    accum_error_     << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    previous_pose_   << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    Kp_ << 0.5 , 0.5 , 0.5 , 0.0, 0.0, 0.0;
    Ki_ << 0.02, 0.02, 0.02, 0.0, 0.0, 0.0;
    Kd_ << 0.3 , 0.3 , 0.3 , 0.0, 0.0, 0.0;
    Adaptive_ = false;
    Update_Goal_ = false;
    a0est_ = 0;
    a1est_ = 0;
    a2est_ = 0;
    x_      << 0.0,0.0,0.0;

    double  po,zeta,wn,ts;
    po     = 15.0;
    ts     = 5.0;
    zeta   = sqrt(log(po/100.0)*log(po/100.0)/(PI*PI+log(po/100.0)*log(po/100.0)));
    wn     = 4.0/(ts*zeta);
    A_ <<   0.0, 1.0  , 0.0      ,
            0.0, 0.0  , 1.0      ,
            0.0,-wn*wn,-2*zeta*wn;
    B_ <<   0.0  ,
            0.0  ,
            wn*wn;
    previous_velocity_ = 0;
    return;
}


void PositionCommand::control()
{
    geometry_msgs::Twist        vel_cmd_msg;
    geometry_msgs::Pose         current_error_msg ;
    Eigen::Matrix<double,6,1>   current_error, accum_error_sat;
    double max_I = 2 , min_I = -2;

    current_error = goal_pose_ - current_pose_;

    double  error, error_dot, s, yr, l, k, gama,
            a0est_dot,a1est_dot,a2est_dot,
            ym,ym_dot,ym_ddot,ym_dddot,
            r,yp, yp_dot, yp_ddot;

    if (control_)
    {
        //////////////////////////////////////////////////////////////////////////////////////////////
        // PID Control algorithm:
        accum_error_ = accum_error_+ period_*(current_error + previous_error_)/2.0;
        accum_error_sat(0,0) = Sat(accum_error_(0,0),max_I,min_I);
        accum_error_sat(1,0) = Sat(accum_error_(1,0),max_I,min_I);
        accum_error_sat(2,0) = Sat(accum_error_(2,0),max_I,min_I);
        accum_error_sat(5,0) = Sat(accum_error_(5,0),max_I,min_I);

        Eigen::Matrix<double,6,6> Cp = Kp_ * current_error.transpose();
        Eigen::Matrix<double,6,6> Ci = Ki_ * accum_error_sat.transpose();
        Eigen::Matrix<double,6,6> Cd = Kd_ * (current_error - previous_error_).transpose()/period_;
        Eigen::Matrix<double,6,6> vel_cmd_current = Cp + Ci + Cd ;
        Eigen::Matrix<double,3,1> linear_vel_cmd_l_frame;

        linear_vel_cmd_l_frame << vel_cmd_current(0,0),
                vel_cmd_current(1,1),
                vel_cmd_current(2,2);

        Eigen::Matrix<double,3,1> linear_vel_cmd_g_frame = rot_matrix_.inverse() * linear_vel_cmd_l_frame;
        //////////////////////////////////////////////////////////////////////////////////////////////

        //ros::Time curr_time = ros::Time::now();
        //      time = ros::Time::now().toSec() - time0_.toSec();
        //        if(Update_Goal_)
        //        {
        //            yd      =   a2b_*b_*time*time + b_*time + c_;
        //            yd_dot  = 2*a2b_*b_*time      + b_;
        //            yd_ddot = 2*a2b_*b_;

        //            std::cout   << "yd = "          << yd
        //                        << "\tyd_dot = "    << yd_dot
        //                        << "\tyd_ddot = "   << yd_ddot
        //                        << std::endl;
        //            if (time >= duration_)
        //            {
        //                Update_Goal_ = false;
        //                yd      = a2b_*b_*duration_*duration_ + b_*duration_ + c_;
        //                yd_dot  = 0;
        //                yd_ddot = 0;
        //            }
        //        }

        if (Adaptive_)
        {
            Eigen::Matrix<double,1,3> G;
            G << 10.0 ,5.582 ,1.5317;
            k = 1;
            l = 1;
            gama = 1;
            ROS_INFO("------------------------------------------------");
            r = goal_pose_(0,0);
            r = G(0,0) * r - G*x_;
            r = Sat(r,0.2,-0.2);
            x_dot_ = A_*x_ + B_*r;
            x_(0,0)     = integration (x_(0,0) , x_dot_(0,0)  , period_ );
            x_(1,0)     = integration (x_(1,0) , x_dot_(1,0)  , period_ );
            x_(2,0)     = integration (x_(2,0) , x_dot_(2,0)  , period_ );
            ym          = x_(0,0);      // position
            ym_dot      = x_(1,0);      // velocity
            ym_ddot     = x_(2,0);      // acceleration
            ym_dddot    = x_dot_(2,0);  // acceleration dot
            yp          = current_pose_ (0,0);// position
            yp_dot      = velocity_     (0,0);// velocity
            yp_ddot     = acceleration_ (0,0);// acceleration


//            double current_velocity = (current_pose_ (0,0)-previous_pose_(0,0)/period_);
//            yp_dot      = current_velocity;
//            yp_ddot     =(current_velocity-previous_velocity_/period_);
//            previous_velocity_ = current_velocity;

            error       = yp_dot    - ym_dot;
            error_dot   = yp_ddot   - ym_ddot;
            s           = error_dot + l*error;
            yr          = ym_dddot  - l*error_dot;

            if (fabs(s) < 0.1)
                s = 0;

            if (fabs(error) < 0.1)
                yp_dot = ym_dot;

            if (fabs(error_dot) < 0.1)
                yp_ddot = ym_ddot;

            a0est_dot = - gama * s * yr;
            a1est_dot = - gama * s * yp_ddot;
            a2est_dot = - gama * s * yp_dot;
            a0est_ = integration (a0est_, a0est_dot , period_ );
            a1est_ = integration (a1est_, a1est_dot , period_ );
            a2est_ = integration (a2est_, a2est_dot , period_ );

            linear_vel_cmd_g_frame(0,0) = a0est_*yr + a1est_*yp_ddot + a2est_*yp_dot - k*s;

            current_error_msg.position.x    = a0est_;//current_error(0,0);
            current_error_msg.position.y    = a1est_;//current_error(1,0);
            current_error_msg.position.z    = a2est_;//current_error(2,0);
            current_error_msg.orientation.x = ym;//current_error(0,0);
            current_error_msg.orientation.y = s;//current_error(1,0);
            current_error_msg.orientation.z = error;//current_error(2,0);
            current_error_msg.orientation.w = k*s;//current_error(2,0);

            current_error_pub.publish(current_error_msg);

            std::cout << "CE: ym ="     <<  ym
                      << " a0est_ = "   <<  a0est_
                      << " a1est_ = "   <<  a1est_
                      << " a2est_ = "   <<  a2est_
                      << std::endl;

        }

        vel_cmd_msg.linear.x  = Sat(linear_vel_cmd_g_frame(0,0),1,-1);
        vel_cmd_msg.linear.y  = Sat(linear_vel_cmd_g_frame(1,0),1,-1);
        vel_cmd_msg.linear.z  = Sat(linear_vel_cmd_g_frame(2,0),1,-1);
        vel_cmd_msg.angular.z = Sat(vel_cmd_current(5,5),1,-1);

        previous_error_     = current_error;
        control_            = false;

        vel_cmd.publish(vel_cmd_msg);

        std::cout << "CE: x ="  <<  current_error(0,0)
                  << " y = "    <<  current_error(1,0)
                  << " z = "    <<  current_error(2,0)
                  << " w = "    <<  current_error(5,0)
                  << std::endl;

        std::cout << "vel: x =" <<  vel_cmd_msg.linear.x
                  << " y = "    <<  vel_cmd_msg.linear.y
                  << " z = "    <<  vel_cmd_msg.linear.z
                  << " w = "    <<  vel_cmd_msg.angular.z
                  << std::endl;
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
    previous_pose_ = current_pose_;
    current_pose_ <<    Pose->pose.position.x,
            Pose->pose.position.y,
            Pose->pose.position.z,
            roll,
            pitch,
            yaw;
    control_ = true;

    // if it is the first time: initialize the function
    if(init_)
    {
        ROS_INFO("initialize");
        previous_time_  = ros::Time::now();
        goal_pose_      << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;
        init_           = false;
        control_        = false;
        Adaptive_       = false;
    }

    ros::Time current_time  = Pose->header.stamp;
    period_                 = current_time.toSec() - previous_time_.toSec();
    previous_time_          = current_time;

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

    time0_ = ros::Time::now();
    //    c_ = current_pose_(0,0);
    //    a2b_  = 0.5;
    //    duration_ = 10*fabs(goal_pose_(0,0) + c_);
    //    b_ = (goal_pose_(0,0) - c_ )/ (a2b_*duration_*duration_ + duration_);
    //    Update_Goal_ = true;


    //    std::cout   << "c_ = "          << c_
    //                << "\ta2b_ = "      << a2b_
    //                << "\tduration_ = " << duration_
    //                << "\tb_= "         << b_
    //                << std::endl;
}
void PositionCommand::getIMU(const ardrone_autonomy::Navdata::ConstPtr & Nav)
{
    velocity_       << Nav->vx/1000,Nav->vy/1000,Nav->vz/1000;
    acceleration_   << Nav->ax,Nav->ay,Nav->az;
    Got_IMU_ = true;
}
void PositionCommand::adaptive_enable_func(const std_msgs::Int32::ConstPtr & Enable)
{
    if (Enable->data == 1)
    {
        Adaptive_ = true;
        x_ << current_pose_(0,0),velocity_(0,0),acceleration_(0,0);
    }
    else
    {
        Adaptive_ = false;
    }
}
double Sat (double num, double Max , double Min){
    if (num > Max)      {num = Max;}
    else if (num < Min) {num = Min;}
    return num;
}
double integration (double y, double ydot , double period ){
    return (y + ydot * period);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "Adaptive");
    ros::NodeHandle n;

    ros::Rate       loop_rate(40);
    PositionCommand position_commander(n);

    while (ros::ok())
    {
        position_commander.control();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

