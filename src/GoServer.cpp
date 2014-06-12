#include "ros/ros.h"
#include "uav_commander/GoSrv.h"


void GoFunction(uav_commander::GoSrv::Response &res ,
                uav_commander::GoSrv::RequestType::_x_type x,
                uav_commander::GoSrv::RequestType::_y_type y,
                uav_commander::GoSrv::RequestType::_z_type z,
                uav_commander::GoSrv::RequestType::_w_type w)
{
    res.Goal.pose.position.x = x;
    res.Goal.pose.position.y = y;
    res.Goal.pose.position.z = z;
    res.Goal.pose.orientation.w = w;
    res.Goal.header.stamp = ros::Time::now();
}

bool GoCall(uav_commander::GoSrv::Request  &req,uav_commander::GoSrv::Response &res)
{
//    if  (req.Name == 'Home' || req.Name == 'home')
//        GoFunction(res,0,0,1,0);

//    else if  (req.Name == 'p1' || req.Name == 'P1')
//        GoFunction(res,1,0,1,0);

//    else if  (req.Name == 'p2' || req.Name == 'P2')
//        GoFunction(res,1,1,1,0);

//    else if  (req.Name == 'n'  || req.Name == 'N')
//        GoFunction(res,req.x,req.y,req.z,req.w);
    res.Goal.pose.position.x = req.x;
    res.Goal.pose.position.y = req.y;
    res.Goal.pose.position.z = req.z;
    res.Goal.pose.orientation.w = req.w;
    res.Goal.header.stamp = ros::Time::now();
    ROS_INFO("request: x=%f, y=%f z=%f, w=%f", req.x,req.y,req.z,req.w);

    return true;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "go_server");
    ros::NodeHandle n;
    ros::ServiceServer  service     = n.advertiseService("go_place", GoCall);
    ROS_INFO("Ready to go.");
    ros::spin();

    return 0;
}
