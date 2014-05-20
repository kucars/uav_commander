//#include <ros/ros.h>
//#include <actionlib/client/simple_action_client.h>
//#include <actionlib/client/terminal_state.h>
//#include <uav_commander/WayPointAction.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <Eigen/Eigen>
//int main (int argc, char **argv)
//{
//    int i;
//    double y,x,z,r,pi = 3.14;
//    geometry_msgs::PoseStamped Position[22];
//    y = -1.2;
//    r =  1;
//    for (i=0;i<=21;i++)
//    {
//        z = sin(2*pi*1/40.0*i)+0.5;
//        if (i<=11)
//        {
//            y+= 0.2;
//            x = sqrt(r*r-y*y);
//        }
//        else
//        {
//            y-= 0.2;
//            x = -sqrt(r*r-y*y);
//        }

//        Position[i].pose.position.x =  y+1;
//        Position[i].pose.position.y = -x;
//        Position[i].pose.position.z =  z;
//    }

//    ros::init(argc, argv, "way_point_client");
//    ros::NodeHandle n_;
//    actionlib::SimpleActionClient<uav_commander::WayPointAction> ac("/uav/uav_commander", true);

//    ROS_INFO("Waiting for action server to start.");
//    ac.waitForServer();

//    ROS_INFO("Action server started, sending goal.");

//    uav_commander::WayPointGoal way_point;
//    for (i=0;i<=21;i++)
//    {
//        way_point.goal = Position[i];
//        ac.sendGoal(way_point);
//        bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
//        if (finished_before_timeout)
//        {
//            actionlib::SimpleClientGoalState state = ac.getState();
//            ROS_INFO("The way point number %d has been reached: %s",i,state.toString().c_str());
//        }
//        else
//        {
//            ROS_INFO("The way point has been reached before the time out.");
//            break;
//        }
//        std::cout << "sleeping..." << std::endl;
//         sleep(5.0);
//         std::cout << "waking" << std::endl;
//        //ros::Duration(20.0);
//    }
//    return 0;

//}

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <uav_commander/WayPointAction.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <std_msgs/Empty.h>

void call_square(int* Imax , double *x,double *y,double *z)
{
    int i,imax=9;
    *Imax = imax;
    double X[] = {0.0 , 0.0 , 1.0 , 2.0 , 2.0 , 2.0 , 1.0 , 0.0 , 0.0};
    double Y[] = {0.0 ,-1.0 ,-1.0 ,-1.0 , 0.0 , 1.0 , 1.0 , 1.0 , 0.0};
    double Z[] = {1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0};
    for (i=0;i<imax;i++)
    {
        x[i] = X[i];
        y[i] = Y[i];
        z[i] = Z[i];
    }
}
void call_tri   (int* Imax , double *x,double *y,double *z)
{
    int i,imax=7;
    *Imax = imax;
    double X[] = {0.0 , 0.0 , 1.0 , 2.0 , 1.0 , 0.0 , 0.0};
    double Y[] = {0.0 ,-1.0 ,-0.5 , 0.0 , 0.5 , 1.0 , 0.0};
    double Z[] = {1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0};
    for (i=0;i<imax;i++)
    {
        x[i] = X[i];
        y[i] = Y[i];
        z[i] = Z[i];
    }
}
void call_8shape(int* Imax , double *x,double *y,double *z)
{
    int i,imax=9;
    *Imax = imax;
    double X[] = {0.0 , 0.0 , 1.0 , 2.0 , 2.0 , 2.0 , 1.0 , 0.0 , 0.0};
    double Y[] = {0.0 ,-1.0 , 0.0 , 1.0 , 0.0 ,-1.0 , 0.0 , 1.0 , 0.0};
    double Z[] = {1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0};
    for (i=0;i<imax;i++)
    {
        x[i] = X[i];
        y[i] = Y[i];
        z[i] = Z[i];
    }
}

int main (int argc, char **argv)
{
    int i,j,k;
    geometry_msgs::PoseStamped Home;
    Home.pose.position.x =  0.0;
    Home.pose.position.y =  0.0;
    Home.pose.position.z =  1.0;

    ros::init(argc, argv, "way_point_client");
    ros::NodeHandle n_;
    actionlib::SimpleActionClient<uav_commander::WayPointAction> ac("/uav/uav_commander", true);
    ros::Publisher land_pub = n_.advertise <std_msgs::Empty> ("/ardrone/land" , 1);
    std_msgs::Empty land_msg;

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO("Action server started, sending goal.");

    uav_commander::WayPointGoal way_point;
    way_point.goal = Home;
    int imax,n;
    double x[10],y[10],z[10] ;
        for (k=1;k<4;k++)
    {
        switch (k)
        {
        case 1:
            call_square(&imax,x,y,z);
            break;
        case 2:
            call_tri(&imax,x,y,z);
            break;
        case 3:
            call_8shape(&imax,x,y,z);
            break;
        }

        for (i=1;i<=imax;i++)
        {
            n=0;
            for (j=0;j<2;j++)
            {
                n+=0.5;
                way_point.goal.pose.position.x = (x[i]-x[i-1])*n + x[i-1];
                way_point.goal.pose.position.y = (y[i]-y[i-1])*n + y[i-1];
                way_point.goal.pose.position.z = (z[i]-z[i-1])*n + z[i-1];
                ac.sendGoal(way_point);
                bool finished_before_timeout = ac.waitForResult(ros::Duration(20.0));
                if (finished_before_timeout)
                {
                    actionlib::SimpleClientGoalState state = ac.getState();
                    ROS_INFO("The way point number %d has been reached: %s",i,state.toString().c_str());
                }
                else
                {
                    ROS_INFO("The way point has been reached before the time out.");
                    break;
                }
            }
            sleep(.05);
        }
        std::cout << "sleeping..." << std::endl;
        sleep(5);
        std::cout << "waking" << std::endl;
    }
    for (i=1;i<10;i++)
    {
        land_pub.publish(land_msg);
        sleep(0.1);
    }

    return 0;

}
