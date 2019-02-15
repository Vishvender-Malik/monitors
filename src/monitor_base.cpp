/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_base.cpp
*/

#include "monitor_base.h"

// base class constructor function to initialize parameter server
monitor_base::monitor_base()
{   
    ROS_INFO("start of monitor_base constructor reached\n");
    
    
    //this -> set_monitor_topics(config, level);
    //callback_variable = boost::bind(&monitor_base::set_monitor_topics, this, _1, _2);
    /*
    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    //ros::spin();
    */
    ROS_INFO("end of monitor_base constructor reached\n");
}

monitor_base::~monitor_base()
{
    ROS_INFO("monitor_base destructor called\n");
}
