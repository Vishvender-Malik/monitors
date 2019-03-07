/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_base.cpp
*/

#include "monitor_base.h"

// base class constructor function to initialize parameter server
monitor_base::monitor_base()
{   
    ROS_INFO("monitor_base constructor called\n");
}

void monitor_base::init_parameter_server(){
    ROS_INFO("init_parameter_server function called\n");
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    //dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and bind to it our callback function
    //dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    //this -> set_monitor_topics(config, level);
    callback_variable = boost::bind(&monitor_base::dynamic_reconfigure_callback, this, _1, _2);

    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    //ROS_INFO("Spinning parameter server.\n");
    //ros::spinOnce();
    ROS_INFO("init_parameter_server function ended\n");
}

monitor_base::~monitor_base()
{
    ROS_INFO("monitor_base destructor called\n");
}
