/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : api_pkg_ros_monitor.h
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
#ifndef api_pkg_ros_monitor
#define api_pkg_ros_monitor
class monitor_base
{
    public:
    monitor_base(); // constructor will initialize parameter server
    virtual void set_monitor_topics(pkg_ros_monitor::monitor_Config &config) = 0; // set custom topics
    virtual void initialize_pub_and_sub() = 0; // initialize publishers and subscribers
    // start the monitor (first get monitor instance),
    // will contain monitor logic or a call to logic function
    //virtual void monitor_start() = 0; // "= 0" makes it a pure virtual function, and the class abstract
    //virtual ~monitor_base() {} // destructor 
    
}; // end of class monitor_base

class monitor_wind : public monitor_base
{
    private:
        monitor_wind() {} // private constructor

        // don't forget to declare these two. We want to make sure they
        // are unacceptable otherwise we may accidentally get copies of
        // our singleton appearing.
        monitor_wind(monitor_wind const&); // don't implement
        void operator = (monitor_wind const&); // don't implement

    public:
        //monitor_wind(monitor_wind const&) = delete; // for use with C++11
        //void operator=(monitor_wind const&)  = delete; // for use with C++11
        
        static monitor_wind& getInstance(); // get single instance of this class
        void set_monitor_topics(pkg_ros_monitor::monitor_Config &config); // set custom topics
        void initialize_pub_and_sub(); // initialize publishers and subscribers
        //void monitor_start();

}; // end of class monitor_wind

class monitor_geo_fence
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

