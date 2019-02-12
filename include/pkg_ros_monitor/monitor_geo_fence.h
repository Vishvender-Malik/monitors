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
    private:
        monitor_geo_fence() {} // private constructor
        
        monitor_geo_fence(monitor_geo_fence const&); // don't implement
        void operator = (monitor_geo_fence const&); // don't implement

    public:
        //monitor_geo_fence(monitor_geo_fence const&) = delete; // for use with C++11
        //void operator=(monitor_geo_fence const&)  = delete; // for use with C++11
        static monitor_geo_fence& getInstance(); // get single instance of this class
        //void set_monitor_topics(pkg_ros_monitor::monitor_Config &config); // set custom topics
        //void initialize_pub_and_sub(); // initialize publishers and subscribers
        //void monitor_start();
}; // end of class monitor_geo_fence

#endif
