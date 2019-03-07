/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_base.h
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
#ifndef header_monitor_base
#define header_monitor_base

#include "headers.h"

class monitor_base
{
    public:
    
    static pkg_ros_monitor::monitor_Config &config;
    uint32_t level;
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and provide it info about our callback function
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    
    monitor_base();
    virtual ~monitor_base();

    // initialize parameter server
    void init_parameter_server();
    // callback function to configuration file to load parameters
    virtual void dynamic_reconfigure_callback(pkg_ros_monitor::monitor_Config &config, uint32_t level) = 0;
    // initialize publishers and subscribers
    virtual void initialize_pub_and_sub() = 0; 
    // start the monitor,
    // will contain monitor logic or a call to logic function
    virtual void monitor_start() = 0; // "= 0" makes it a pure virtual function, and the class abstract
        
}; // end of class monitor_base

#endif
