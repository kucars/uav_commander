#include "position_command.h"

Position_Command::Position_Command(ros::NodeHandle & n,std::string name) :
    n_(n), action_name_(name),goal_received_(false),pose_received_(false) {
    ROS_INFO("Initialization Position_Command");

    pose_sub        = n_.subscribe("pose"   , 1, &Position_Command::getPose   , this);
    goal_sub        = n_.subscribe("goal"   , 1, &Position_Command::getGoal   , this);
    vel_cmd_pub     = n_.advertise <geometry_msgs::Twist       > ("/cmd_vel"  , 1);
    previous_time_  = ros::Time::now();
    as_(n, name, false);

//  Change to read from Yamal file
    max_vel_        =  1;
    min_vel_        = -1;
    goal_sphere_    = 0.2;
    goal_pose_      << 0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

    return;
}

void Position_Command::getGoal(const  geometry_msgs::PoseStamped::ConstPtr  & Goal){
    ROS_INFO("GOT NEW GOAL");
    goal_received_  = true;
    goal_pose_      = ConvertPose(Goal);
    std::cout << "goal_pose: " << goal_pose_.transpose() << std::endl;
}

void Position_Command::getPose(const geometry_msgs::PoseStamped::ConstPtr   & Pose){
    pose_received_  = true;
    previous_pose_  = current_pose_;
    current_pose_   = ConvertPose(Pose);

    ros::Time current_time  = ros::Time::now();
    period_                 = current_time.toSec() - previous_time_.toSec();
    previous_time_          = current_time;
}

Eigen::Matrix<double,3,1> Position_Command::ConvertPose (const geometry_msgs::PoseStamped::ConstPtr & Pose){
    Eigen::Matrix<double,6,1> position;
    Eigen::Matrix<double,3,1> euler = Eigen::Quaterniond(Pose->pose.orientation.w,
                                                         Pose->pose.orientation.x,
                                                         Pose->pose.orientation.y,
                                                         Pose->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    position << Pose->pose.position.x,
                Pose->pose.position.y,
                Pose->pose.position.z,
                euler(0,0),
                euler(1,0),
                euler(2,0);

    return position;
}

bool Position_Command::CheckGoal(const Eigen::Matrix<double,6,1> e){
    bool result = false;
    error       = sqrt(e.transpose() * e);
    if (error <= goal_sphere_)
        result = true;
    else
        return result;
}

double Position_Command::sat (double num, double Max , double Min){
    if (num > Max)
    {
        num = Max;
    }
    else if (num < Min)
    {
        num = Min;
    }
    return num;
}

double Position_Command::velocity_sat (double num){
    return sat ( num, max_vel_ , min_vel_ );
}

