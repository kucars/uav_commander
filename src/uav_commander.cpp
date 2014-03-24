/***************************************************************************
* Copyright (C) 2013 - 2014 by                                             *
* Tarek Taha, Khalifa University Robotics Institute KURI                   *
*                     <tarek.taha@kustar.ac.ae>                            *
*                                                                          *
*                                                                          *
* This program is free software; you can redistribute it and/or modify     *
* it under the terms of the GNU General Public License as published by     *
* the Free Software Foundation; either version 2 of the License, or        *
* (at your option) any later version.                                      *
*                                                                          *
* This program is distributed in the hope that it will be useful,          *
* but WITHOUT ANY WARRANTY; without even the implied warranty of           *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
* GNU General Public License for more details.                             *
*                                                                          *
* You should have received a copy of the GNU General Public License        *
* along with this program; if not, write to the                            *
* Free Software Foundation, Inc.,                                          *
* 51 Franklin Steet, Fifth Floor, Boston, MA 02111-1307, USA.              *
***************************************************************************/
#include "ros/ros.h"
#include "std_msgs/String.h"
//#include "pctx_control/Control.h"
#include "visualeyez_tracker/TrackerPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include <Eigen/Eigen>
#include <std_msgs/String.h>

#include <sstream>
#define PI 3.14159265
double x,y,z;

const double epsilon=10.0;

inline bool equalFloat(double a, double b, double epsilon)
{
    return fabs(a - b) < epsilon;
}

class PositionCommand
{
public:
    // PID gains
    Eigen::Matrix<double,6,1> Kp;
    Eigen::Matrix<double,6,1> Kd;
    Eigen::Matrix<double,6,1> Ki;

    std::string base_marker_id_;
    std::string head_marker_id_;
    double freq_;
    bool got_pose_update_;
    bool goal_;
    bool got_base_marker_;
    bool got_head_marker_;

    Eigen::Vector3d base_marker_position;
    Eigen::Vector3d head_marker_position;

    Eigen::Matrix<double,6,1> previous_pose_;
    Eigen::Matrix<double,6,1> current_pose_;

    ros::Subscriber goal_sub;
    ros::Subscriber markers_sub;
    ros::Publisher vel_cmd;


    ros::NodeHandle n_;

    Eigen::Matrix<double,6,1> goal_pose_;


    PositionCommand(ros::NodeHandle & n, std::string & base_marker_id, std::string & head_marker_id, double & freq,
                    Eigen::Matrix<double,6,1> & kp) :
        n_(n),
        base_marker_id_(base_marker_id),
        head_marker_id_(head_marker_id),
        freq_(freq),
        Kp(kp),
        goal_(false)
    {
        ROS_INFO_STREAM("base marker id: "<<base_marker_id_);
        ROS_INFO_STREAM("head marker id: "<<head_marker_id_);
        ROS_INFO_STREAM("freq: "<<freq_);


        goal_sub = n_.subscribe("Goal", 1000, &PositionCommand::getGoal, this);

        markers_sub = n_.subscribe("TrackerPosition", 1000, &PositionCommand::getUpdatedPose, this);
        vel_cmd = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

private:
    void getUpdatedPose(const visualeyez_tracker::TrackerPose::ConstPtr& trackerPose);
    void getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal);
};

void PositionCommand::getUpdatedPose(const visualeyez_tracker::TrackerPose::ConstPtr& trackerPose)
{
    //ROS_DEBUG(" Recieved Tracker Location: [%s] [%f] [%f] [%f]",trackerPose.tracker_id.c_str(),trackerPose.pose.x ,trackerPose.pose.y ,trackerPose.pose.z );

    if(!goal_)
        return;
    if(trackerPose->tracker_id==base_marker_id_)
    {
        base_marker_position=Eigen::Vector3d(trackerPose->pose.x,trackerPose->pose.y,trackerPose->pose.z);
        got_base_marker_=true;
    }
    else if(trackerPose->tracker_id==head_marker_id_)
    {
        head_marker_position=Eigen::Vector3d(trackerPose->pose.x,trackerPose->pose.y,trackerPose->pose.z);
        got_head_marker_=true;
    }

    // If both markers available, do the control
    if(got_base_marker_ && got_head_marker_)
    {
        // Define a local reference frame
        Eigen::Vector3d uav_x=(head_marker_position-base_marker_position).normalized(); // Heading

        Eigen::Vector3d uav_y=(uav_x.cross(Eigen::Vector3d::UnitZ())).normalized();
        // LETS ASSUME THE UAV Z POINTS ALWAYS UP


        Eigen::Matrix<double, 3, 3> current_rotation_matrix;
        current_rotation_matrix << uav_x, uav_y, Eigen::Vector3d::UnitZ();
        Eigen::Matrix<double, 3, 1> current_euler = current_rotation_matrix.eulerAngles(2, 1, 0);

        double yaw = current_euler(0,0);
        double pitch = current_euler(1,0);
        double roll = current_euler(2,0);

        current_pose_ << base_marker_position.x(),
                base_marker_position.y(),
                base_marker_position.z(),
                roll,
                pitch,
                yaw;

        Eigen::Matrix<double,6,1> error=goal_pose_-current_pose_;
        // Check if goal was reached
        if(equalFloat(current_pose_(0,0), goal_pose_(0,0),epsilon),
                equalFloat(current_pose_(1,0), goal_pose_(1,0),epsilon),
                equalFloat(current_pose_(2,0), goal_pose_(2,0),epsilon),
                equalFloat(current_pose_(3,0), goal_pose_(3,0),epsilon),
                equalFloat(current_pose_(4,0), goal_pose_(4,0),epsilon),
                equalFloat(current_pose_(5,0), goal_pose_(5,0),epsilon)
                )
            //if(error.norm()<epsilon) // CHANGE THIS
        {
            ROS_INFO("Reached goal!");
            goal_=false;
        }
        else //control
        {
            ROS_INFO_STREAM("position error norm: "<< error.transpose().norm());

            Kp << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0; //P gain
            Eigen::Matrix<double,6,1> Kd;
            Kp << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0; //P gain
            Eigen::Matrix<double,6,6> vel_cmd_input=Kp*(current_pose_-previous_pose_).transpose();

            geometry_msgs::Twist vel_cmd_msg;
            vel_cmd_msg.linear.x=vel_cmd_input(0,0);
            vel_cmd_msg.linear.y=vel_cmd_input(1,1);
            vel_cmd_msg.linear.z=vel_cmd_input(2,2);
            vel_cmd_msg.angular.x=vel_cmd_input(3,3);
            vel_cmd_msg.angular.y=vel_cmd_input(4,4);
            vel_cmd_msg.angular.z=vel_cmd_input(5,5);
            vel_cmd.publish(vel_cmd_msg);
        }

        previous_pose_=current_pose_;
    }
}

void PositionCommand::getGoal(const geometry_msgs::PoseStamped::ConstPtr & goal)
{
    ROS_INFO("GOT NEW GOAL");
    Eigen::Matrix<double,3,1> euler=Eigen::Quaterniond(goal->pose.orientation.w,
                                                       goal->pose.orientation.x,
                                                       goal->pose.orientation.y,
                                                       goal->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    double yaw = euler(0,0);
    double pitch = euler(1,0);
    double roll = euler(2,0);

    goal_pose_ << goal->pose.position.x,
            goal->pose.position.y,
            goal->pose.position.z,
            roll,
            pitch,
            yaw;
    goal_=true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;
    ros::NodeHandle n_priv("~");
    std::string base_marker_id;
    std::string head_marker_id;
    double freq;

    n_priv.param<double>("freq", freq, 50.0);
    n_priv.param<std::string>("base_marker_id", base_marker_id, "teste");
    n_priv.param<std::string>("head_marker_id", head_marker_id, "teste2");

    ///////////
    // Gains //
    ///////////


    // Proportional (kd)
    double kp_x;
    double kp_y;
    double kp_z;
    double kp_roll;
    double kp_pitch;
    double kp_yaw;

    n_priv.param<double>("kp_x", kp_x, 1.0);
    n_priv.param<double>("kp_y", kp_y, 1.0);
    n_priv.param<double>("kp_z", kp_z, 1.0);
    n_priv.param<double>("kp_roll", kp_roll, 0.0);
    n_priv.param<double>("kp_pitch", kp_pitch, 0.0);
    n_priv.param<double>("kp_yaw", kp_yaw, 1.0);

    Eigen::Matrix<double,6,1> Kp;


    PositionCommand position_commander(n,base_marker_id,head_marker_id, freq, Kp);
    ros::spin();
    return 0;
    /*//ros::Publisher uav_commands = n.advertise<pctx_control::Control>("sendPCTXControl", 1000);
    //ros::Subscriber sub = n.subscribe("TrackerPosition", 1000, getUpdatedPose);
    ros::Rate loop_rate(15);
    int count = 0;
    std::vector<int16_t> controlValues(9,0);
    double k = 3;
    while (ros::ok())
    {
        pctx_control::Control controlMessage;
        int val = count%1020;//int(sin((count%180)*PI/180.0)*1020);
        //controlValues[0] = controlValues[1] =controlValues[2] =controlValues[3] =controlValues[4] =controlValues[5] =controlValues[6] =controlValues[7] =controlValues[8] = val;
        controlValues[0] = k*y;
        controlMessage.values  = controlValues;
        ROS_DEBUG("UAV_Commander broadcasting to all channels value:%d",controlValues[0]);
        uav_commands.publish(controlMessage);
        ros::spinOnce();
        loop_rate.sleep();
        count+=10;
    }*/
}
