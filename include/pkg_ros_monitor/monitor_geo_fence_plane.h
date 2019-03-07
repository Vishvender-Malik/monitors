/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : monitor_geo_fence_plane.h
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
#ifndef header_monitor_wind
#define header_monitor_wind

#include "monitor_base.h"

class monitor_geo_fence_plane : public monitor_base
{
    public:
        monitor_geo_fence_plane();
        
        using monitor_base::dynamic_reconfigure_callback;
        virtual void dynamic_reconfigure_callback(pkg_ros_monitor::monitor_Config &config, uint32_t level); // set custom topics
        using monitor_base::initialize_pub_and_sub;
        virtual void initialize_pub_and_sub(); // initialize publishers and subscribers
        using monitor_base::monitor_start;
        virtual void monitor_start();      
}; // end of class monitor_geo_fence_plane

#endif