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
    pkg_ros_monitor::monitor_Config config;
    // constructor will initialize parameter server
    monitor_base(); // can't initialize so can't declare here
    //~monitor_base();
    // callback function to configuration file to load parameters
    virtual void set_monitor_topics(pkg_ros_monitor::monitor_Config &config, uint32_t level){};
    // initialize publishers and subscribers
    virtual void initialize_pub_and_sub(){}; 
    // start the monitor (first get monitor instance),
    // will contain monitor logic or a call to logic function
    virtual void monitor_start(){}; // "= 0" makes it a pure virtual function, and the class abstract
        
}; // end of class monitor_base

#endif
