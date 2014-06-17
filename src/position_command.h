#ifndef POSITION_COMMAND_H
#define POSITION_COMMAND_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

class Position_Command
{
private:
    bool    goal_received_  ,pose_received_ ;
    double  goal_sphere_    ,max_vel_  ,min_vel_;
    Eigen::Matrix<double,6,1> goal_pose_,previous_pose_,current_pose_;
    std::string     action_name_;
    ros::Time       previous_time_;
    ros::NodeHandle n_;
    ros::Subscriber goal_sub,pose_sub;
    ros::Publisher  vel_cmd_pub;

    void getGoal    (const  geometry_msgs::PoseStamped::ConstPtr    & Goal);
    void getPose    (const  geometry_msgs::PoseStamped::ConstPtr    & Pose);

public:
    Position_Command                        (ros::NodeHandle & n,std::string name);
    Eigen::Matrix<double,3,1> ConvertPose   (const geometry_msgs::PoseStamped::ConstPtr & Pose);
    bool    CheckGoal                       (const Eigen::Matrix<double,6,1> e)
    double  velocity_sat                    (double num);
    double  Sat                             (double num, double Max, double Min);

};

#endif // POSITION_COMMAND_H


