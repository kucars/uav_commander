
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
#include <Eigen/Geometry>
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
//double integration (double y, double ydot , double period );
double filter (double lmdaf,double yf,double u, double period);
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
    double  a2b_,duration_,b_,c_,C1,C2;

    Eigen::Matrix<double,6,1> goal_pose_, current_pose_, previous_pose_, accum_error_, previous_error_;
    Eigen::Matrix<double,3,3> rot_matrix_;
    Eigen::Matrix<double,3,1> acceleration_, velocity_,W_,Aest_dot_,Aest_,sf_,sf_dot_;
    Eigen::Matrix<double,6,1> Kp_, Kd_, Ki_;
    Eigen::Matrix<double,2,1> xm_, xm_dot_,Bm_;
    Eigen::Matrix<double,2,2> Am_;
    Eigen::Matrix<double,3,3> P_,P_dot_;
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

    double  po,zeta,wn,ts;
    po     = 15.0;
    ts     = 5.0;
    zeta   = sqrt(log(po/100.0)*log(po/100.0)/(PI*PI+log(po/100.0)*log(po/100.0)));
    wn     = 4.0/(ts*zeta);
    Am_ <<  0.0  , 1.0, -wn*wn,-2*zeta*wn;
    Bm_ <<  0.0  , wn*wn;
    //Aest_       << 0.58 , 0.14 , -0.21;
    Aest_       << 1.0 , 1.0 , 1.0;
    Aest_       <<  0.00542645 ,0.0103635 , 0.784819;
    Aest_dot_   << 0.0 , 0.0 , 0.0;
    sf_         << 0.0 , 0.0 , 0.0;
    sf_dot_     << 0.0 , 0.0 , 0.0;
    P_          << 1.0 , 0.0 , 0.0 ,0.0 , 1.0 , 0.0 ,0.0 , 0.0 , 1.0;
    P_dot_      << 0.0 , 0.0 , 0.0 ,0.0 , 0.0 , 0.0 ,0.0 , 0.0 , 0.0;
    ROS_INFO("initialization");
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
            ym,ym_dot,ym_ddot,
            r,yp, yp_dot;
    if (control_)
    {
        //////////////////////////////////////////////////////////////////////////////////////////////
        // PID Control algorithm:
        accum_error_            = accum_error_+ period_*(current_error + previous_error_)/2.0;
        accum_error_sat(0,0)    = Sat(accum_error_(0,0),max_I,min_I);
        accum_error_sat(1,0)    = Sat(accum_error_(1,0),max_I,min_I);
        accum_error_sat(2,0)    = Sat(accum_error_(2,0),max_I,min_I);
        accum_error_sat(5,0)    = Sat(accum_error_(5,0),max_I,min_I);

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

        if (Adaptive_)
        {

            double k=3.0, l=2.0 , gama=1.0, k0=5.0 , lmda0=10.0 , lf=1.0, u=0.0 , e=0.0, lamda_t=0.0 , SEnable=0.0;
            Eigen::Matrix<double,2,1> xm_dot_new;
            Eigen::Matrix<double,3,3> Af,Bf,P_dot_new;
            Eigen::Matrix<double,3,1> YP,W,Phi,Aest_dot_new,sf_dot_new;
            Af << -lf , 0.0 , 0.0 ,0.0 , -lf , 0.0 ,0.0 , 0.0 , -lf;
            Bf <<  lf , 0.0 , 0.0 ,0.0 ,  lf , 0.0 ,0.0 , 0.0 ,  lf;
            ROS_INFO("--------------- Adaptive ---------------------");
            double G[] = {10.0 ,5.582 ,1.5317};
            r = goal_pose_(2,0);
            r = G[0] * r - G[0]*current_pose_ (2,0) - G[1]*velocity_ (2,0) - G[2]*acceleration_(2,0);
            r = Sat(r,0.5,-0.5);
            //            if (C2 < 5)
            //            {
            //                if (C1 > 80)
            //                {
            //                    C1 = 0.0;
            //                    C2++;
            //                }
            //                else
            //                    C1++;
            //                if (C1 < 40)
            //                    r =  0.1;
            //                else
            //                    r = -0.1;
            //            }
            //            else
            //                r = Sat(goal_pose_(0,0),0.2,-0.2);

            // Refernce Model:-------------------------------------------------------------------------------
            xm_dot_new  = Am_*xm_ + Bm_*r;
            xm_         =     xm_ + period_*(xm_dot_new + xm_dot_)/2.0;
            xm_dot_     = xm_dot_new;
            ym          = xm_(0,0);             // velocity
            ym_dot      = xm_(1,0);             // acceleration
            ym_ddot     = xm_dot_(1,0);         // acceleration dot
            yp          = velocity_     (2,0);  // velocity
            yp_dot      = acceleration_ (2,0);  // acceleration
            // Error:----------------------------------------------------------------------------------------
            error       = yp        - ym;
            error_dot   = yp_dot    - ym_dot;
            s           = error_dot + l*error;
            yr          = ym_ddot   - l*error_dot;
            // Robustness : ---------------------------------------------------------------------------------
            //s = SEnable*s;
            if (fabs(s) < 0.1)
                s = 0;

            if (fabs(error) < 0.1)
                yp = ym;

            if (fabs(error_dot) < 0.1)
                yp_dot = ym_dot;

            //if (fabs(error_dot) < 0.1)
            //    yr = ym_ddot;
            // Filter : -------------------------------------------------------------------------------------
            //            YP << yp,yp_dot,u;
            //            sf_dot_new  = Af * sf_ + Bf * YP;
            //            sf_         = sf_ + period_*(sf_dot_new + sf_dot_)/2.0;
            //            sf_dot_     = sf_dot_new;
            //            // Indirect Adaptive : --------------------------------------------------------------------------
            //            W << lf *(yp_dot - sf_(1,0)),sf_(1,0),sf_(0,0);
            //            e = W.transpose()* Aest_ - sf_(2,0);
            //            // Calculate P matrix : -------------------------------------------------------------------------
            //            lamda_t     = lmda0 *(1 - P_.norm()/k0) ;
            //            P_dot_new   = lamda_t*P_ - P_*( W * W.transpose() ) * P_;
            //            P_          = P_ + + period_*(P_dot_new + P_dot_)/2.0;
            //            P_dot_      = P_dot_new;
            // Adaptive law : -------------------------------------------------------------------------------
            Phi  << yr , yp_dot , yp;
            //Aest_dot_new = - gama * P_ * ( (Phi * s) + ( W * e ));
            Aest_dot_new = - gama * P_ * (Phi * s);
            Aest_  = Aest_ + period_*(Aest_dot_new + Aest_dot_)/2.0;
            Aest_dot_ = Aest_dot_new;
            // Control law : --------------------------------------------------------------------------------
            u = Aest_.transpose() * Phi - k * s;
            linear_vel_cmd_g_frame(2,0) = Sat(u,1,-1);
            if (Sat(u,1,-1)== u )
                SEnable = 1.0;
            else
                SEnable = 0.0;
            // Finish Adaptive Control
            //----------------------------------------------------------------------------------------------

            current_error_msg.position.x    = Aest_(0,0);//current_error(0,0);
            current_error_msg.position.y    = Aest_(1,0);//current_error(1,0);
            current_error_msg.position.z    = Aest_(2,0);//current_error(2,0);
            current_error_msg.orientation.x = ym;//current_error(0,0);
            current_error_msg.orientation.y = r;//current_error(1,0);
            current_error_msg.orientation.z = error;//current_error(2,0);
            current_error_msg.orientation.w = acceleration_ (0,0);//current_error(2,0);
            std::cout << " ym ="     <<  ym
                      << " r = "     <<  r
                      << " error = " <<  error
                      << " s = "     <<  s
                      << " u = "     <<  u
                      << " A0 = "    <<  Aest_(0,0)
                      << " A1 = "    <<  Aest_(1,0)
                      << " A2 = "    <<  Aest_(2,0)
                      << std::endl;
            current_error_pub.publish(current_error_msg);
        }

        vel_cmd_msg.linear.x  = Sat(linear_vel_cmd_g_frame  (0,0) ,1,-1);
        vel_cmd_msg.linear.y  = Sat(linear_vel_cmd_g_frame  (1,0) ,1,-1);
        vel_cmd_msg.linear.z  = Sat(linear_vel_cmd_g_frame  (2,0) ,1,-1);
        vel_cmd_msg.angular.z = Sat(vel_cmd_current         (5,5) ,1,-1);

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
    current_pose_ <<
                     Pose->pose.position.x,
            Pose->pose.position.y,
            Pose->pose.position.z,
            roll,
            pitch,
            yaw;
    control_ = true;

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
}
void PositionCommand::getIMU(const ardrone_autonomy::Navdata::ConstPtr & Nav)
{
    //double yf =  acceleration_(0,0);
    velocity_       <<  Nav->vx/1000,Nav->vy/1000,Nav->vz/1000;
    acceleration_   <<  Nav->ax*9.8 ,Nav->ay*9.8 ,(Nav->az-1)*9.8 ;
    //acceleration_(0,0) = low_BF (yf,acceleration_(0,0),period_);
    Got_IMU_ = true;
}
void PositionCommand::adaptive_enable_func(const std_msgs::Int32::ConstPtr & Enable)
{
    if (Enable->data == 1)
    {
        Adaptive_ = true;
        xm_         << velocity_(0,0)     ,acceleration_(0,0);
        xm_dot_     << acceleration_(0,0) , 0.0 ;
    }
    else
    {
        Adaptive_ = false;
        ROS_INFO("--------------------------------------------------------");
        std::cout << "A1 ="     <<  Aest_(0,0)
                  << " A2 = "   <<  Aest_(1,0)
                  << " A3 = "   <<  Aest_(2,0)
                  << std::endl;
        ROS_INFO("--------------------------------------------------------");
    }
}
double Sat (double num, double Max , double Min){
    if (num > Max)      {num = Max;}
    else if (num < Min) {num = Min;}
    return num;
}
//void integration (  Eigen::Matrix<double,2,1> *y        , Eigen::Matrix<double,2,1> ydot_new,
//                    Eigen::Matrix<double,2,1> ydot_old  , Eigen::Matrix<double,2,1> period )
//{
//    return (y+ period*(ydot_new + ydot_old)/2.0);
//}

double filter (double lmdaf,double yf,double u, double period){
    double f = 1;
    double ydot = - f*yf + f*u;
    yf = yf + ydot*period;
    return yf;
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

