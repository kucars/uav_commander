#include "ros/ros.h"
#include "uav_commander/GoSrv.h"
#include <cstdlib>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_client");
    if (argc != 5)
    {
        ROS_INFO("usage: go(x,y,z,w)");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient  client      = n.serviceClient   <uav_commander::GoSrv>      ("go_place");
    ros::Publisher      goal_pub    = n.advertise       <geometry_msgs::PoseStamped>("/uav/goal",1);

    uav_commander::GoSrv srv;
    geometry_msgs::PoseStamped goal_msg;

    srv.request.x   = atoll(argv[1]);
    srv.request.y   = atoll(argv[2]);
    srv.request.z   = atoll(argv[3]);
    srv.request.w   = atoll(argv[4]);
    goal_msg.pose.position.x      = atoll(argv[1]);
    goal_msg.pose.position.y      = atoll(argv[2]);
    goal_msg.pose.position.z      = atoll(argv[3]);
    goal_msg.pose.orientation.w   = atoll(argv[4]);
    goal_msg.header.stamp         = ros::Time::now();
    std::cout << "going to pub" << std::endl;

    goal_pub.publish(goal_msg);
    ros::spinOnce();
}
