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
    private:
        //monitor_wind() {} // private constructor

        // don't forget to declare these two. We want to make sure they
        // are unacceptable otherwise we may accidentally get copies of
        // our singleton appearing.
        //monitor_wind(monitor_wind const&); // don't implement
        //void operator = (monitor_wind const&); // don't implement

    public:
        //monitor_wind(monitor_wind const&) = delete; // for use with C++11
        //void operator=(monitor_wind const&)  = delete; // for use with C++11
        monitor_wind();
        //static monitor_wind& getInstance(); // get single instance of this class
        void set_monitor_topics(pkg_ros_monitor::monitor_Config &config, uint32_t level); // set custom topics
        void initialize_pub_and_sub(); // initialize publishers and subscribers
        void monitor_start();

}; // end of class monitor_wind

#endif