#include "PositionCommand.h"
#include "uav_commander/WayPointAction.h"
#include <actionlib/goal_id_generator.h>
PositionCommand::PositionCommand(ros::NodeHandle & n,std::string name) :
    ph_("~"),
    goal_received_(false),
    pose_received_(false),
    action_recived_(false),
    action_name_(name),
    as_(n, name, false)
{
    ROS_INFO("Initialization Position Command");

    pose_sub        = n.subscribe("/uav/pose"   , 1, &PositionCommand::getPose   , this);
    goal_sub        = n.subscribe("goal"   , 1, &PositionCommand::getGoal   , this);
    curr_error_pub  = n.advertise <geometry_msgs::Pose > ("/current_error"  , 1);
    previous_time_  = ros::Time::now();

    as_.registerGoalCallback   (boost::bind(&PositionCommand::goalCB, this));
    as_.start();

    dynamic_function = boost::bind(&PositionCommand::dynamic,this, _1, _2);
    server.setCallback(dynamic_function);

    goal_pose_ << 0.0 ,0.0 ,0.0 ,0.0 ,0.0 ,0.0;
    std::cout<< "Read Parameters" << std::endl;

    pidGainLoad(pid_x_ , x_gain_ , ros::NodeHandle(ph_, "PIDx"));
    pidGainLoad(pid_y_ , y_gain_ , ros::NodeHandle(ph_, "PIDy"));
    pidGainLoad(pid_z_ , z_gain_ , ros::NodeHandle(ph_, "PIDz"));
    pidGainLoad(pid_w_ , w_gain_ , ros::NodeHandle(ph_, "PIDw"));

    ph_.param<double>("goal_sphere" ,goal_sphere_       ,0.2);
    ph_.param<double>("home_pose/x" ,goal_pose_(0,0)    ,0.0);
    ph_.param<double>("home_pose/y" ,goal_pose_(1,0)    ,0.0);
    ph_.param<double>("home_pose/z" ,goal_pose_(2,0)    ,1.0);

    std::cout<< "goal_sphere_ = "   << goal_sphere_             << std::endl;
    std::cout<< "home_pose = "      << goal_pose_.transpose()   << std::endl;
    return;
}
void PositionCommand::pidGainLoad (control_toolbox::Pid &pid , control_toolbox::Pid::Gains & G, const ros::NodeHandle &n )
{
    double Kp,Ki,Kd,IMax,IMin;
    n.param<double>("Kp"     ,G.p_gain_  , 0.0);
    n.param<double>("Ki"     ,G.i_gain_  , 0.0);
    n.param<double>("Kd"     ,G.d_gain_  , 0.0);
    n.param<double>("IMax"   ,G.i_max_   , 0.0);
    n.param<double>("IMin"   ,G.i_min_   , 0.0);

    std::cout   << "Kp = "      << G.p_gain_
                << " Ki = "     << G.i_gain_
                << " Kd = "     << G.d_gain_
                << " IMax = "   << G.i_max_
                << " IMin = "   << G.i_min_
                << std::endl;

    pid.setGains(Kp,Ki,Kd,IMax,IMin);
}

void PositionCommand::getGoal(const  geometry_msgs::PoseStamped::ConstPtr  & Goal)
{
    ROS_INFO("GOT NEW GOAL");
    goal_received_  = true;
    convertPose(Goal,goal_pose_);
    std::cout << "goal_pose: " << goal_pose_.transpose() << std::endl;
}

void PositionCommand::getPose(const geometry_msgs::PoseStamped::ConstPtr   & Pose)
{
    previous_pose_  = current_pose_;
    convertPose ( Pose, current_pose_);

    ros::Time current_time  = ros::Time::now();
    duration_               = current_time - previous_time_;
    if (duration_.toSec() < 0.1)
        pose_received_  = true;

    return;
}

void PositionCommand::convertPose (const geometry_msgs::PoseStamped::ConstPtr & Pose , Eigen::Matrix<double,6,1> & Converted_Pose)
{

    Eigen::Matrix<double,3,1> euler = Eigen::Quaterniond(Pose->pose.orientation.w,
                                                         Pose->pose.orientation.x,
                                                         Pose->pose.orientation.y,
                                                         Pose->pose.orientation.z).matrix().eulerAngles(2, 1, 0);
    Converted_Pose <<   Pose->pose.position.x,
            Pose->pose.position.y,
            Pose->pose.position.z,
            euler(0,0),
            euler(1,0),
            euler(2,0);

    return;
}

bool PositionCommand::checkGoal(const Eigen::Matrix<double,6,1> e){
    bool result = false;

    double error = sqrt(e.transpose() * e);

    if (error <= goal_sphere_)
        result = true;
    else
        return result;
}

void PositionCommand::PIDControl()
{
    ros::Time Now = ros::Time::now();
    geometry_msgs::Twist vel;

    if (pose_received_)
    {
        current_error_ = goal_pose_ - current_pose_;
        vel.linear.x  = pid_x_.computeCommand(current_error_(0,0),duration_);
        vel.linear.y  = pid_y_.computeCommand(current_error_(1,0),duration_);
        vel.linear.z  = pid_z_.computeCommand(current_error_(2,0),duration_);
        vel.angular.z = pid_w_.computeCommand(current_error_(5,0),duration_);
        interface_.velCom(vel);
        last_time_pub_ = Now;
        pose_received_ = false;
    }

    else if(Now > last_time_pub_ + ros::Duration(1.0))
    {
        interface_.velCom(vel);
        last_time_pub_ = Now;
    }
    return;
}

void PositionCommand::goalCB()
{
    geometry_msgs::PoseStamped Pt_G = as_.acceptNewGoal()->goal;
    //convertPose(,goal_pose_);

    ROS_INFO("Got New WayPoint from Action client");
    std::cout << "goal_pose: " << goal_pose_.transpose()<< std::endl;
    goal_received_  = true;
    action_recived_ = true;
    return;
}

void PositionCommand::PubError (Eigen::Matrix<double,6,1> E)
{
    geometry_msgs::Pose error;
    error.position.x    = E(0,0);
    error.position.y    = E(1,0);
    error.position.z    = E(2,0);
    error.orientation.x = E(0,0);
    error.orientation.y = E(1,0);
    error.orientation.z = E(2,0);
    curr_error_pub.publish(error);
}

void PositionCommand::Print ()
{
    std::cout << "Pose: x ="  <<  current_pose_(0,0)
              << " y = "    <<  current_pose_(1,0)
              << " z = "    <<  current_pose_(2,0)
              << " w = "    <<  current_pose_(5,0)
              << std::endl;

    std::cout << "Goal: x ="  <<  goal_pose_(0,0)
              << " y = "    <<  goal_pose_(1,0)
              << " z = "    <<  goal_pose_(2,0)
              << " w = "    <<  goal_pose_(5,0)
              << std::endl;

    std::cout << "Error: x ="  <<  current_error_(0,0)
              << " y = "    <<  current_error_(1,0)
              << " z = "    <<  current_error_(2,0)
              << " w = "    <<  current_error_(5,0)
              << std::endl;

    //    std::cout << "vel: x =" <<  vel_cmd_msg.linear.x
    //              << " y = "    <<  vel_cmd_msg.linear.y
    //              << " z = "    <<  vel_cmd_msg.linear.z
    //              << " w = "    <<  vel_cmd_msg.angular.z
    //              << std::endl;
}

void PositionCommand::dynamic(uav_commander::position_command_dynamic_parmsConfig &config, uint32_t level)
{
    control_toolbox::Pid::Gains G;

    if (config.Load_PID)
    {
        pidGainLoad(pid_x_ , x_gain_ , ros::NodeHandle(ph_, "PIDx"));
        pidGainLoad(pid_y_ , y_gain_ , ros::NodeHandle(ph_, "PIDy"));
        pidGainLoad(pid_z_ , z_gain_ , ros::NodeHandle(ph_, "PIDz"));
        pidGainLoad(pid_w_ , w_gain_ , ros::NodeHandle(ph_, "PIDw"));

        config.Kp_x = x_gain_.p_gain_;
        config.Ki_x = x_gain_.i_gain_;
        config.Kd_x = x_gain_.d_gain_;

        config.Kp_y = y_gain_.p_gain_;
        config.Ki_y = y_gain_.i_gain_;
        config.Kd_y = y_gain_.d_gain_;

        config.Kp_z = z_gain_.p_gain_;
        config.Ki_z = z_gain_.i_gain_;
        config.Kd_z = z_gain_.d_gain_;

        config.Kp_w = w_gain_.p_gain_;
        config.Ki_w = w_gain_.i_gain_;
        config.Kd_w = w_gain_.d_gain_;

        config.Load_PID = 0;
    }
    control_enable_ = config.PID_Control;

    if (config.Take_off)
    {
        config.Take_off = 0;
        interface_.takeOff();
    }
    if (config.Land)
    {
        config.Land = 0;
        interface_.land();
    }
    if (config.Reset)
    {
        config.Reset = 0;
        interface_.emergency();
    }
    if (config.Send_Position)
    {
        goal_received_  = true;
        goal_pose_ << config.X_Position,config.Y_Position,config.Z_Position,0,0,0;
        config.Send_Position = 0;
        ROS_INFO("GOT NEW GOAL FROM THE GUI");
        std::cout << "goal_pose: " << goal_pose_.transpose()<< std::endl;
    }
    pid_x_.setGains(config.Kp_x,config.Ki_y,config.Kd_x,x_gain_.i_max_,x_gain_.i_min_);
    pid_y_.setGains(config.Kp_y,config.Ki_y,config.Kd_y,y_gain_.i_max_,y_gain_.i_min_);
    pid_z_.setGains(config.Kp_z,config.Ki_z,config.Kd_z,z_gain_.i_max_,z_gain_.i_min_);
    pid_w_.setGains(config.Kp_w,config.Ki_w,config.Kd_w,w_gain_.i_max_,w_gain_.i_min_);

}

void PositionCommand::actionRecive()
{
    feedback_.pose.pose.position.x = current_pose_(0,0);
    feedback_.pose.pose.position.y = current_pose_(1,0);
    feedback_.pose.pose.position.z = current_pose_(2,0);
    feedback_.pose.pose.orientation.x = current_pose_(3,0);
    feedback_.pose.pose.orientation.y = current_pose_(4,0);
    feedback_.pose.pose.orientation.z = current_pose_(5,0);
    as_.publishFeedback(feedback_);

    if(checkGoal(current_error_))
    {
        res_.pose = feedback_.pose;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        as_.setSucceeded(res_);
        action_recived_ = false;
    }
}

void PositionCommand::enable_control()
{
    ros::Time current_time  = ros::Time::now();

    if ((previous_time_.toSec() - current_time.toSec()) < 1.0)
    {
        geometry_msgs::Twist vel;
        interface_.velCom(vel);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "uav_commander");
    ros::NodeHandle n;

    ros::Rate       loop_rate(40);
    PositionCommand position_commander(n,ros::this_node::getName());

    while (ros::ok())
    {
        // position_commander.enable_control();
        // if (position_commander.action_recived_)
        //     position_commander.actionRecive();

        if (position_commander.control_enable_)
            position_commander.PIDControl();

        position_commander.Print();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
