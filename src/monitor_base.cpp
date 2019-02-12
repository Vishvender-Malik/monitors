/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_base.cpp
*/

#include "monitor_base.h"

// base class constructor function to initialize parameter server
monitor_base::monitor_base()
{   
    ROS_INFO("start of monitor_base constructor reached\n");
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and provide it info about our callback function
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    callback_variable = boost::bind(&set_monitor_topics, _1, _2);

    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    //ros::spin();
    ROS_INFO("end of monitor_base constructor reached\n");
}

