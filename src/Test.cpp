#include "ros/ros.h"
#include "Teleop_UAV.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");
    ros::NodeHandle n,n_priv("~");
    Teleop_UAV teleop_uav(n,n_priv);
    ros::Rate  loop_rate(40);

    boost::thread thread_watchdog   (boost::bind(&Teleop_UAV::watchdog, &teleop_uav));
    boost::thread thread_key        (boost::bind(&Teleop_UAV::keyLoop,  &teleop_uav));

    double x;
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    teleop_uav.quit();
    ros::shutdown();
    exit(0);
}
