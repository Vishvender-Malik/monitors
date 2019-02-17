/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_wind.h
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
#ifndef header_monitor_wind
#define header_monitor_wind

#include "monitor_base.h"

class monitor_wind : public monitor_base
{
    public:
        //monitor_wind(monitor_wind const&) = delete; // for use with C++11
        //void operator=(monitor_wind const&)  = delete; // for use with C++11
        monitor_wind();
        //static monitor_wind& getInstance(); // get single instance of this class
        //virtual void init_parameter_server();
        using monitor_base::set_monitor_topics;
        virtual void set_monitor_topics(pkg_ros_monitor::monitor_Config &config, uint32_t level); // set custom topics
        using monitor_base::initialize_pub_and_sub;
        virtual void initialize_pub_and_sub(); // initialize publishers and subscribers
        using monitor_base::monitor_start;
        virtual void monitor_start();      
}; // end of class monitor_wind

#endif