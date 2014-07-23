#ifndef POSITIONCOMMAND_H
#define POSITIONCOMMAND_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "PositionCommand.h"

#include <sstream>
#include <sys/time.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <control_toolbox/pid.h>
#include "ardrone_interface.h"
#include <actionlib/server/simple_action_server.h>
#include "uav_commander/WayPointAction.h"
//#include <actionlib/goal_id_generator.h>
#include <Eigen/Eigen>
#include <uav_commander/position_command_dynamic_parmsConfig.h>
#include <tf/tf.h>

class PositionCommand
{
private:
    bool    goal_received_ ;
    bool    pose_received_ ;
    double  goal_sphere_   ;

    ArdroneInterface interface_;
    control_toolbox::Pid pid_x_;
    control_toolbox::Pid pid_y_;
    control_toolbox::Pid pid_z_;
    control_toolbox::Pid pid_w_;

    Eigen::Matrix<double,6,1> goal_pose_;
    Eigen::Matrix<double,6,1> previous_pose_;
    Eigen::Matrix<double,6,1> current_pose_;
    Eigen::Matrix<double,6,1> current_error_;

    ros::Time       previous_time_;
    ros::Time       last_time_pub_;
    ros::Duration   duration_;
    ros::NodeHandle ph_;
    ros::Subscriber goal_sub,pose_sub;
    ros::Publisher  vel_cmd_pub,curr_error_pub;
    control_toolbox::Pid::Gains x_gain_, y_gain_,z_gain_, w_gain_;

    actionlib::SimpleActionServer <uav_commander::WayPointAction> as_;
    uav_commander::WayPointFeedback feedback_;
    uav_commander::WayPointResult   res_;
    std::string action_name_;

    dynamic_reconfigure::Server<uav_commander::position_command_dynamic_parmsConfig> server;
    dynamic_reconfigure::Server<uav_commander::position_command_dynamic_parmsConfig>::CallbackType dynamic_function;

    void getGoal    (const  geometry_msgs::PoseStamped::ConstPtr    & Goal);
    void getPose    (const  geometry_msgs::PoseStamped::ConstPtr    & Pose);
    void pidGainLoad (control_toolbox::Pid &pid , control_toolbox::Pid::Gains &G, const ros::NodeHandle &n);
public:
    bool    control_enable_;
    bool    action_recived_;

    PositionCommand         (ros::NodeHandle & n, std::string name);
    void    convertPose     (const geometry_msgs::PoseStamped::ConstPtr & Pose , Eigen::Matrix<double,6,1> & Converted_Pose);
    bool    checkGoal       (const Eigen::Matrix<double,6,1> e);
    void    PIDControl      ();
    void    goalCB          ();
    void    PubError        (Eigen::Matrix<double,6,1> E);
    void    Print           ();
    void    dynamic         (uav_commander::position_command_dynamic_parmsConfig &config, uint32_t level);
    void    actionRecive    ();
    void    enable_control  ();
};


#endif // POSITIONCOMMAND_H


