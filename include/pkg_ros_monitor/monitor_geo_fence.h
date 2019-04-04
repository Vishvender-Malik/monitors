/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_geo_fence.h
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
#ifndef header_monitor_geo_fence
#define header_monitor_geo_fence

#include "monitor_base.h"

class monitor_geo_fence : public monitor_base
{
    public:
        //monitor_geo_fence(monitor_geo_fence const&) = delete; // for use with C++11
        //void operator=(monitor_geo_fence const&)  = delete; // for use with C++11
        monitor_geo_fence();
        using monitor_base::dynamic_reconfigure_callback;
        virtual void dynamic_reconfigure_callback(pkg_ros_monitor::monitor_Config &config, uint32_t level); // set custom topics
        using monitor_base::initialize_pub_and_sub;
        virtual void initialize_pub_and_sub(); // initialize publishers and subscribers
        virtual void monitor_logic();
}; // end of class monitor_geo_fence

#endif
