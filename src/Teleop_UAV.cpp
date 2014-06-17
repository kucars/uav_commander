#include "Teleop_UAV.h"

Teleop_UAV::Teleop_UAV(ros::NodeHandle & n,ros::NodeHandle & n_priv):n_(n),n_priv_(n_priv),hovering_(false),watch_dog_(false),kfd_(0)
{
    vel_cmd_pub         = n_.advertise      <geometry_msgs::Twist> 	("/cmd_vel"    		, 1);
    takeoff_pub         = n_.advertise      <std_msgs::Empty >      ("/ardrone/takeoff"	, 1);
    land_pub            = n_.advertise      <std_msgs::Empty >      ("/ardrone/land"   	, 1);
    reset_pub           = n_.advertise      <std_msgs::Empty >      ("/ardrone/reset"  	, 1);
    flattrim_client_    = n_.serviceClient  <std_srvs::Empty>       ("/ardrone/flattrim");
    navdata_sub_        = n_.subscribe      ("/ardrone/navdata"   , 1, &Teleop_UAV::get_navigation    , this);

    n_priv_.param<double>("x_scale",x_scale_,0);
    n_priv_.param<double>("y_scale",y_scale_,0);
    n_priv_.param<double>("z_scale",z_scale_,0);
    n_priv_.param<double>("w_scale",w_scale_,0);
    n_priv_.param<double>("emergency_battery_level",emergency_battery_level_,100);
    n_priv_.param<double>("take_off_time",take_off_time_,0);

    tcgetattr(kfd_, &cooked_);
    memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &=~ (ICANON | ECHO);
    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    tcsetattr(kfd_, TCSANOW, &raw_);

    // Description:
    {
        std::cout << "Reading from keyboard"                                        << std::endl;
        std::cout <<"----------------------------------------------------------"    << std::endl;
        std::cout <<" Use the following keys to control the ardrone"                << std::endl;
        std::cout <<" f            : Flat Trim"                                     << std::endl;
        std::cout <<" l            : Land"                                          << std::endl;
        std::cout <<" t            : Take off"                                      << std::endl;
        std::cout <<" e            : Toggle Emergency Light"                        << std::endl;
        std::cout <<" w            : Move Forward"                                  << std::endl;
        std::cout <<" s            : Move Backward"                                 << std::endl;
        std::cout <<" d            : Move Right"                                    << std::endl;
        std::cout <<" a            : Move Left"                                     << std::endl;
        std::cout <<" q    	    : Quit the code"                                    << std::endl;
        std::cout <<" Up Arrow     : Move up"                                       << std::endl;
        std::cout <<" Down Arrow   : Move down"                                     << std::endl;
        std::cout <<" Right Arrow  : Rotoate right"                                 << std::endl;
        std::cout <<" Left Arrow   : Rotoate Left"                                  << std::endl;
    }
}   

void Teleop_UAV::keyLoop()
{
    ros::Rate  loop_rate(40);
    while (ros::ok())
    {
        if (!keyread())
            break;
        ros::spinOnce();
        loop_rate.sleep();
    }
    quit();
    ros::shutdown();
    exit(0);
}

bool Teleop_UAV::keyread()
{

    int x = 0, y = 0, z = 0 , w = 0;
    bool state = true , send_vel = false;

    if(read(kfd_, &c_, 1) < 0)
    {
        perror("read():");
        exit(-1);
    }

    switch(c_)
    {
    case Arrow_U    :   z =  1.0;       send_vel = true;    break;
    case Arrow_D    :   z = -1.0;       send_vel = true;    break;
    case Arrow_L    :   w =  1.0;       send_vel = true;    break;
    case Arrow_R    :   w = -1.0;       send_vel = true;    break;
    case KEY_w      :   x =  1.0;       send_vel = true;    break;
    case KEY_s      :   x = -1.0;       send_vel = true;    break;
    case KEY_d      :   y =  1.0;       send_vel = true;    break;
    case KEY_a      :   y = -1.0;       send_vel = true;    break;
    case KEY_f      :   flat_trim();                        break;
    case KEY_l      :   land();                             break;
    case KEY_EMR    :   land();                             break;
    case KEY_t      :   take_off();                         break;
    case KEY_e      :   emergency();                        break;
    case KEY_q      :   state = false;                     break;
    }
    if (send_vel && hovering_)
    {
        move ( x , y , z , w);
        watch_dog_      = false;
        last_publish_   = ros::Time::now();
    }
    return state;
}

void Teleop_UAV::take_off ()
{
    flat_trim ();
    std::cout << "Taking Off" << std::endl;
    std_msgs::Empty msg;
    takeoff_pub.publish(msg);
    check_hovering();
}

void Teleop_UAV::emergency()
{
    std::cout << "Toggle Emergency" << std::endl;
    std_msgs::Empty msg;
    reset_pub.publish(msg);
}

void Teleop_UAV::land ()
{
    std::cout << "Landding" << std::endl;
    std_msgs::Empty msg;
    land_pub.publish(msg);
}

void Teleop_UAV::flat_trim ()
{
    std::cout << "Flat Trim" << std::endl;
    std_srvs::Empty srv;
    flattrim_client_.call(srv);
}

void Teleop_UAV::check_hovering(){
    ros::Time time0 = ros::Time::now();
    while (1)
    {
        if(navigation_data_.state == 4)
        {
            std::cout << "Ardrone is Hovering" << std::endl;
            hovering_ = true;
            break;
        }
        if(ros::Time::now() > time0 + ros::Duration(take_off_time_))
        {
            land ();
            hovering_ = false;
            break;
        }
    }
}

void Teleop_UAV::move (int x, int y, int z, int w)
{
    geometry_msgs::Twist vel_msg;
    std::cout << "Moving" << std::endl;
    vel_msg.linear.x    =  x * x_scale_;
    vel_msg.linear.y    =  y * y_scale_;
    vel_msg.linear.z    =  z * z_scale_;
    vel_msg.angular.z   =  w * w_scale_;
    vel_cmd_pub.publish(vel_msg);
}

void Teleop_UAV::watchdog()
{
    geometry_msgs::Twist vel_msg;
    while(ros::ok())
    {
        if (ros::Time::now() > last_publish_ + ros::Duration(0.5) && !watch_dog_ && hovering_)
        {
            watch_dog_ = true;
            std::cout << "Hovering zero velocity" << std::endl;
            last_publish_ = ros::Time::now();
            vel_cmd_pub.publish(vel_msg);
        }
    }
    ros::shutdown();
    exit(0);
}

void Teleop_UAV::quit()
{
    tcsetattr(kfd_, TCSANOW, &cooked_);
}

void Teleop_UAV::get_navigation(const ardrone_autonomy::Navdata::ConstPtr & Nav)
{
    navigation_data_ = *Nav;
    if (navigation_data_.batteryPercent < emergency_battery_level_)
        land();
}
