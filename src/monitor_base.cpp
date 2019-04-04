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

void monitor_base::monitor_init(){

    ROS_INFO("start of monitor_init function reached");
    // get single wind monitor instance
    //monitor_wind::getInstance().initialize_pub_and_sub();
    //ROS_INFO("monitor_wind::getInstance function executed");
    //monitor_wind::initialize_pub_and_sub();

    //run loop at (10) Hz (always in decimal and faster than what is published through guidance controller)
    ros::Rate loop_rate(1.00);
    
    // Use current time as seed for random generator 
    srand(time(0)); 

    while (ros::ok())
    {
        // keep calling parameter server callback function to check for changes in parameters
        // kind of redundant to keep calling it since the parameters are set the first time it is called before
        // but here it is called to display ROS_INFO about the parameters, otherwise ROS_INFO is displayed only once in the 
        // beginning of the output
        //parameter_server.setCallback(callback_variable); 
        //monitor_wind::initialize_pub_and_sub();
        ros::spinOnce(); // if we have subscribers in our node, but always keep for good measure
        
        this -> monitor_logic(); // keep calling this function
        
        // sleep for appropriate time to hit mark of (10) Hz
        loop_rate.sleep();
    } // end of while loop
    ROS_INFO("end of monitor_init function reached");
}

monitor_base::~monitor_base()
{
    ROS_INFO("monitor_base destructor called\n");
}
