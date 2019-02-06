/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : node_pkg_ros_monitor.cpp
*/

#include <iostream>
#include <string>
#include <cmath>
#include <ctime>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Point32.h>
#include <dynamic_reconfigure/server.h>
#include <pkg_ros_monitor/monitor_Config.h>
#include "TargetInfo.h"
#include "Contour.h"
#include "api_pkg_ros_monitor.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/State.h>

//-------------------------------------------------------------------------------------------------------------------------------

//<-------------------------------------------Global variables and structures--------------------------------------------------->

// for wind monitor
std::string topic_guidance_velocity = "";
std::string topic_desired_airspeed = "";
std::string topic_receive_altitude = "";
std::string topic_wind_estimation = "";
std::string topic_blobDataFlags = "";
std::string topic_corrected_airspeed = "";
std::string topic_local_position_data = "";

// for geo fence monitor
std::string topic_corrected_velocity = ""; 
std::string topic_new_mavros_state = "";
std::string topic_current_mavros_state = "";

const int array_velocity_guidance_size = 3, array_wind_velocity_size = 3, array_guidance_velocity_size = 3,
array_total_irl_vel_size = 3, size_vel_difference = 3, size_covariance = 36, array_new_airspeed_size = 3,
array_local_position_pose_data_size = 3, monitor_geo_fence_triggered_size = 3;

std::string array_monitor_geo_fence_triggered[monitor_geo_fence_triggered_size] = "No"; // for geo fence monitor
std::string mode_flight = "";

double array_velocity_guidance[array_velocity_guidance_size] = {0}, array_windspeed[array_wind_velocity_size] = {0},
array_groundspeed_guidance[array_guidance_velocity_size] = {0}, array_actual_groundspeed[array_total_irl_vel_size] = {0},
array_vel_difference[size_vel_difference] = {0}, array_new_airspeed[array_new_airspeed_size] = {0},
array_local_position_pose_data[array_local_position_pose_data_size] = {0},
max_possible_pose_in_positive_x = 0.0, max_possible_pose_in_negative_x = 0.0, max_possible_pose_in_positive_y = 0.0, 
max_possible_pose_in_negative_y = 0.0, max_possible_pose_in_positive_z = 0.0, max_possible_pose_in_negative_z = 0.0,
fence_limit_to_consider_in_x, fence_limit_to_consider_in_y, fence_limit_to_consider_in_z,
dist_bet_fence_and_vehicle_x = 0.0, dist_bet_fence_and_vehicle_y = 0.0, dist_bet_fence_and_vehicle_z = 0.0,
critical_radius_from_fence_limit, radius_of_circle_of_influence_s, dist_bet_fence_and_vehicle_overall = 0.0,
angle_bet_fence_and_vehicle = 0.0, gradient_x = 0.0, gradient_y = 0.0, constant_beta = 0.0, resulting_velocity_of_vehicle = 0.0;

int flag_target_detected, flag_target_tracking, first_choice = 0, second_choice = 0,
sign_vehicle_pose_x, sign_vehicle_pose_y, sign_vehicle_pose_z;

float altitude = 0.0, array_covariance[size_covariance], sign_groundspeed_guidance_vel_x, sign_groundspeed_guidance_vel_y, 
sign_groundspeed_guidance_vel_z, sign_actual_groundspeed_vel_x, sign_actual_groundspeed_vel_y, sign_actual_groundspeed_vel_z, 
max_possible_vel_in_positive_x, max_possible_vel_in_positive_y, max_possible_vel_in_z, max_possible_vel_in_negative_x, 
max_possible_vel_in_negative_y, max_possible_vel_in_positive_z, max_possible_vel_in_negative_z, 
sign_new_airspeed_vel_x, sign_new_airspeed_vel_y,
sign_new_airspeed_vel_z, sign_local_position_y;

float target_center_x, target_center_y;

bool mode_guided;

geometry_msgs::TwistStamped command_geometry_twist; // final command_geometry_twist message to be published
nav_msgs::Odometry command_nav_pose; // final command_nav_pose message to be published
mavros_msgs::State command_mavros_state; // final command_mavros_state to be published

// publishers and subscribers for monitor_wind
ros::Publisher pub_landing_target_info;
ros::Publisher pub_corrected_airspeed;
ros::Subscriber sub_groundspeed_guidance;
ros::Subscriber sub_guidance_velocity;
ros::Subscriber sub_receive_altitude;
ros::Subscriber sub_get_windspeed;
ros::Subscriber sub_vision_landing_target_info;

// publishers and subscribers for monitor_wind
ros::Publisher pub_corrected_velocity;
ros::Publisher pub_new_mavros_state;
ros::Subscriber sub_local_position_data;
ros::Subscriber sub_current_mavros_state;

//<----------------------------------------------------------------------------------------------------------->

//<------------------------------------------Local function declarations--------------------------------------------------------->

void reconfiguration_callback(pkg_ros_monitor::monitor_Config &config, uint32_t level);
void set_topic_guidance_velocity(std::string guidance_velocity);
void set_topic_desired_airspeed(std::string desired_airspeed);
void set_topic_receive_altitude(std::string receive_altitude);
void set_topic_wind_estimation(std::string wind_estimation);
void set_topic_blobDataFlags(std::string blobDataFlags);
void set_topic_corrected_airspeed(std::string corrected_airspeed);
void receive_guidance_velocity(const geometry_msgs::TwistStamped::ConstPtr &data);
void receiveGroundspeedGuidance(const geometry_msgs::TwistStamped::ConstPtr &data);
void receiveAltitude(const std_msgs::Float64::ConstPtr &data);
void getWindspeed(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr &data);
void vision_landing_target_info(const pkg_ros_monitor::TargetInfo::ConstPtr &info);
void prediction_from_monitor_wind();
void publish_final_command_wind();
void publish_final_command_geo_fence();
void monitor_wind_start();
void monitor_geo_fence_start();
void set_max_possible_pose_in_positive_x(int config_max_possible_pose_in_positive_x);
void set_max_possible_pose_in_negative_x(int config_max_possible_pose_in_negative_x);
void set_max_possible_pose_in_positive_y(int config_max_possible_pose_in_positive_y);
void set_max_possible_pose_in_negative_y(int config_max_possible_pose_in_negative_y);
void set_max_possible_pose_in_positive_z(int config_max_possible_pose_in_positive_z);
void set_max_possible_pose_in_negative_z(int config_max_possible_pose_in_negative_z);
void set_new_mavros_state(std::string new_mavros_state);
void set_topic_local_position_data(std::string local_position_data);
void set_topic_corrected_velocity(std::string corrected_velocity);
void set_constant_beta(double config_constant_beta);
void set_critical_radius_from_fence_limit(double critical_radius);
void set_radius_of_circle_of_influence_s(double radius_of_circle_of_influence);
void receive_local_position_data(const nav_msgs::Odometry::ConstPtr &data);
//void receive_current_mavros_state(const mavros_msgs::State::ConstPtr &data);
void prediction_from_monitor_geo_fence();

//<------------------------------------------------------------------------------------------------------------------------>

//<------------------------------------------------Initialize monitor node--------------------------------------------------->

int main(int argc, char **argv)
{
    ROS_INFO("\n\n----------------Welcome-----------------\n\n");

    // initialize ros node with a node name
    ros::init(argc, argv, "node_pkg_ros_monitor");
    
    std::cout<<"Monitors available : \n\n"
    "1. Wind monitor.\n"
    "2. Geo fence monitor.\n"
    "3. Exit.\n\n"
    "Kindly enter your choice (number) to start a monitor : ";

    std::cin>> first_choice;
    std::cout<<"\nYou have enetered the choice : "<<first_choice<<".\n\n";

    switch (first_choice)
    {
        case 1: std::cout<<"Wind monitor has started.\n\n";
        //monitor_wind::getInstance(); 
        monitor_wind_start(); break;
        case 2: std::cout<<"Geo fence monitor has started.\n\n"; 
        monitor_geo_fence_start();
        break;
        case 3: std::cout<<"Monitor has exited.\n\n"; return 0;
        default: std::cout<<"This is the default choice. Monitor application has now been shut down.\n\n"; return 0;
    }
    /* for when we figure out how to output to a new terminal
    while(second_choice != 2 && second_choice != 3)
    {
        std::cout<<"Would you like to start another monitor? \n\n"
        "1. Yes please.\n"
        "2. No, thank you. Just keep running current monitor(s).\n"
        "3. No thank you. Exit the whole monitor application.\n\n";

        std::cin>> second_choice;
        std::cout<<"\nYou have enetered the choice : "<<second_choice<<".\n\n";

        switch (second_choice)
        {
            case 1: 
            {   std::cout<<"Monitors available : \n\n"
                "1. Wind monitor.\n"
                "2. Some other monitor.\n"
                "3. Exit.\n\n"
                "Kindly enter your choice to start a monitor : ";

                std::cin>> first_choice;
                std::cout<<"\nYou have enetered the choice : "<<first_choice<<".\n\n";

                switch (first_choice)
                {
                    case 1: std::cout<<"Wind monitor has started.\n";
                    monitor_wind::monitor_wind_start(); break;
                    case 2: std::cout<<"Unfortunately, there's no other monitor available at the moment. Monitor application has exited.\n\n"; return 0;
                    case 3: std::cout<<"Monitor has exited.\n\n"; return 0;
                    default: std::cout<<"This is the default first_choice. Monitor application has exited.\n\n"; return 0;
                }
            }
            case 2: std::cout<<"Sure thing. Just press Ctrl + C to shut down each monitor in their terminal. Menu will exit now.\n\n"; break;
            case 3: std::cout<<"Monitor application has now been shut down.\n\n"; return 0;
            default: std::cout<<"This is the default choice. Monitor application has now been shut down.\n\n"; return 0;
        } // end of inner switch
    } // end of while
    */
    ROS_INFO("Spinning node");
    ros::spin();

} // end of initialization

//<------------------------------------------Function definitions------------------------------------------------------>

///////////////////////////// Start of monitor_wind code /////////////////////////////////////////////////////////////////

// base class constructor function to initialize parameter server
monitor_base::monitor_base()
{   ROS_INFO("start of monitor_base constructor reached\n");
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and provide it info about our callback function
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    callback_variable = boost::bind(&reconfiguration_callback, _1, _2);

    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    //ros::spin();
    ROS_INFO("end of monitor_base constructor reached\n");
}

// function to set topics for monitor_wind
void monitor_wind::set_monitor_topics(pkg_ros_monitor::monitor_Config &config)
{
    ROS_INFO("start of monitor_wind::set_monitor_topics function reached\n");
    set_topic_guidance_velocity(config.set_topic_guidance_velocity.c_str());
    set_topic_desired_airspeed(config.set_topic_desired_airspeed.c_str());
    set_topic_receive_altitude(config.set_topic_receive_altitude.c_str());
    set_topic_wind_estimation(config.set_topic_wind_estimation.c_str());
    set_topic_blobDataFlags(config.set_topic_blobDataFlags.c_str());
    set_topic_corrected_airspeed(config.set_topic_corrected_airspeed.c_str());
    
    ROS_INFO("Current wind monitor configuration parameters: \n\n""Topic to get velocity parameters for comparison from : %s\n"
    "Topic to get desired airspeed velocity parameters from : %s\n"
    "Topic to get altitude information from : %s\n"
    "Topic to get wind information from : %s\n"
    "Topic to get additional flags information from : %s\n"
    "Topic to publish final corrected airspeed to : %s\n", 
    config.set_topic_guidance_velocity.c_str(), config.set_topic_desired_airspeed.c_str(),
    config.set_topic_receive_altitude.c_str(), config.set_topic_wind_estimation.c_str(),
    config.set_topic_blobDataFlags.c_str(), config.set_topic_corrected_airspeed.c_str());
    ROS_INFO("end of monitor_wind::set_monitor_topics function reached\n");
}

// function to initialize publishers and subscribers for monitor_wind
void monitor_wind::initialize_pub_and_sub()
{
    ROS_INFO("start of initialize pub and sub function reached\n");
    // create a nodehandle to enable interaction with ros commands, usually always just after ros::init
    ros::NodeHandle nodeHandle;
    
    // create publisher and subscriber objects

    // final publisher to application // topic should just be cmd_vel
    pub_corrected_airspeed = nodeHandle.advertise<geometry_msgs::TwistStamped>(topic_corrected_airspeed, 1000); // should be cmd_vel?
    //pub_corrected_airspeed = config.pub_corrected_airspeed; // should be cmd_vel?
    // subscriber to intercept velocity commands from guidance controller
    //sub_groundspeed_guidance = nodeHandle.subscribe("/mavros/setpoint_velocity/cmd_vel_new", 1000, receiveGroundspeedGuidance);
    sub_groundspeed_guidance = nodeHandle.subscribe(topic_guidance_velocity, 1000, receiveGroundspeedGuidance);
    // subscriber to receive velocity commands from the topic itself
    sub_guidance_velocity = nodeHandle.subscribe(topic_desired_airspeed, 1000, receive_guidance_velocity);
    // subscriber to receive altitude data from the topic itself
    sub_receive_altitude = nodeHandle.subscribe(topic_receive_altitude, 1000, receiveAltitude);
    // subscriber to get wind data information
    sub_get_windspeed = nodeHandle.subscribe(topic_wind_estimation, 1000, getWindspeed);
    // subscriber to receive landing target info from the vision controller
    sub_vision_landing_target_info = nodeHandle.subscribe(topic_blobDataFlags, 1000, vision_landing_target_info);
    // sub_vision_landing_target_info = nodeHandle.subscribe("/landing_target_info_new", 10, vision_landing_target_info); // original
    ROS_INFO("end of initialize pub and sub function reached\n");
}

// function to get a single instance of wind monitor
monitor_wind& monitor_wind::getInstance()
{
    static monitor_wind monitor_wind_instance; // instantiated on first use, guaranteed to be destroyed
    ROS_INFO("monitor_wind::getInstance() function reached\n");
    return monitor_wind_instance;
}

// function to initialize all parameters for wind monitor
void monitor_wind_start()
{   ROS_INFO("start of monitor_wind_start function reached");
    // get single wind monitor instance
    //monitor_wind::getInstance().initialize_pub_and_sub();
    ROS_INFO("monitor_wind::getInstance function executed");
    //monitor_wind::initialize_pub_and_sub();

    // create a nodehandle to enable interaction with ros commands, usually always just after ros::init
    ros::NodeHandle nodeHandle;
    
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and provide it info about our callback function
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    callback_variable = boost::bind(&reconfiguration_callback, _1, _2);

    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    
    // create publisher and subscriber objects

    // final publisher to application // topic should just be cmd_vel
    pub_corrected_airspeed = nodeHandle.advertise<geometry_msgs::TwistStamped>(topic_corrected_airspeed, 1000); // should be cmd_vel?
    //pub_corrected_airspeed = config.pub_corrected_airspeed; // should be cmd_vel?
    // subscriber to intercept velocity commands from guidance controller
    //sub_groundspeed_guidance = nodeHandle.subscribe("/mavros/setpoint_velocity/cmd_vel_new", 1000, receiveGroundspeedGuidance);
    sub_groundspeed_guidance = nodeHandle.subscribe(topic_guidance_velocity, 1000, receiveGroundspeedGuidance);
    // subscriber to receive velocity commands from the topic itself
    sub_guidance_velocity = nodeHandle.subscribe(topic_desired_airspeed, 1000, receive_guidance_velocity);
    // subscriber to receive altitude data from the topic itself
    sub_receive_altitude = nodeHandle.subscribe(topic_receive_altitude, 1000, receiveAltitude);
    // subscriber to get wind data information
    sub_get_windspeed = nodeHandle.subscribe(topic_wind_estimation, 1000, getWindspeed);
    // subscriber to receive landing target info from the vision controller
    sub_vision_landing_target_info = nodeHandle.subscribe(topic_blobDataFlags, 1000, vision_landing_target_info);
    // sub_vision_landing_target_info = nodeHandle.subscribe("/landing_target_info_new", 10, vision_landing_target_info); // original

    //run loop at (10) Hz (always in decimal and faster than what is published through guidance controller)
    ros::Rate loop_rate(10);
    
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
        
        publish_final_command_wind(); // keep calling this function
        
        // sleep for appropriate time to hit mark of (10) Hz
        loop_rate.sleep();
    } // end of while loop
    ROS_INFO("end of monitor_wind_start function reached");
} // end of function monitor_wind::monitor_wind_start

// function to set configuration parameters set by user
void reconfiguration_callback(pkg_ros_monitor::monitor_Config &config, uint32_t level) 
{
    // load up custom parameters set by user in the single configuration
    // file for each  individual monitor
    // user can set custom topics at runtime (from terminal)
    //monitor_wind::getInstance().set_monitor_topics(config);
    
    ROS_INFO("reconfiguration callback function reached\n");
    
    // for wind monitor
    set_topic_guidance_velocity(config.set_topic_guidance_velocity.c_str());
    set_topic_desired_airspeed(config.set_topic_desired_airspeed.c_str());
    set_topic_receive_altitude(config.set_topic_receive_altitude.c_str());
    set_topic_wind_estimation(config.set_topic_wind_estimation.c_str());
    set_topic_blobDataFlags(config.set_topic_blobDataFlags.c_str());
    set_topic_corrected_airspeed(config.set_topic_corrected_airspeed.c_str());

    ROS_INFO("Current wind monitor configuration parameters: \n\n"
    "Topic to get velocity parameters for comparison from : %s\n"
    "Topic to get desired airspeed velocity parameters from : %s\n"
    "Topic to get altitude information from : %s\n"
    "Topic to get wind information from : %s\n"
    "Topic to get additional flags information from : %s\n"
    "Topic to publish final corrected airspeed to : %s\n\n", 
    config.set_topic_guidance_velocity.c_str(), config.set_topic_desired_airspeed.c_str(),
    config.set_topic_receive_altitude.c_str(), config.set_topic_wind_estimation.c_str(),
    config.set_topic_blobDataFlags.c_str(), config.set_topic_corrected_airspeed.c_str());

    // for geo fence monitor
    set_max_possible_pose_in_positive_x(config.set_max_possible_pose_in_positive_x);
    set_max_possible_pose_in_negative_x(config.set_max_possible_pose_in_negative_x);
    set_max_possible_pose_in_positive_y(config.set_max_possible_pose_in_positive_y);
    set_max_possible_pose_in_negative_y(config.set_max_possible_pose_in_negative_y);
    set_max_possible_pose_in_positive_z(config.set_max_possible_pose_in_positive_z);
    set_max_possible_pose_in_negative_z(config.set_max_possible_pose_in_negative_z);
    set_critical_radius_from_fence_limit(config.set_critical_radius_from_fence_limit);
    set_radius_of_circle_of_influence_s(config.set_radius_of_circle_of_influence_s);
    set_constant_beta(config.set_constant_beta);
    set_new_mavros_state(config.set_topic_new_mavros_state);
    set_topic_local_position_data(config.set_topic_local_position_data.c_str());
    set_topic_corrected_velocity(config.set_topic_corrected_velocity.c_str());

    ROS_INFO("Current geo fence configuration parameters: \n\n"
    "Fence limit set in positive x : %f\n"
    "Fence limit set in negative x : %f\n"
    "Fence limit set in positive y : %f\n"
    "Fence limit set in negative y : %f\n"
    "Fence limit set in positive z : %f\n"
    "Fence limit set in negative z : %f\n"
    "Critical radius from fence limit : %f\n"
    "Radius of circle of influence \"s\" : %f\n"
    "Value for constant beta (for potential field calculation) : %f\n"
    "Topic to publish new mavros state parameters to : %s\n"
    "Topic to get local position data from : %s\n"
    "Topic to get desired velocity parameters from : %s\n"
    "Topic to publish corrected velocity to : %s \n\n",
    config.set_max_possible_pose_in_positive_x, config.set_max_possible_pose_in_negative_x,
    config.set_max_possible_pose_in_positive_y, config.set_max_possible_pose_in_negative_y,
    config.set_max_possible_pose_in_positive_z, config.set_max_possible_pose_in_negative_z,
    config.set_critical_radius_from_fence_limit, config.set_radius_of_circle_of_influence_s,
    config.set_constant_beta,
    config.set_topic_new_mavros_state.c_str(),
    config.set_topic_local_position_data.c_str(),
    config.set_topic_desired_airspeed.c_str(),
    config.set_topic_corrected_velocity.c_str());
}

// function to set guidance velocity topic
void set_topic_guidance_velocity(std::string guidance_velocity)
{
    topic_guidance_velocity = guidance_velocity;
}

// function to set desired airspeed topic
void set_topic_desired_airspeed(std::string desired_airspeed)
{
    topic_desired_airspeed = desired_airspeed;
}

// function to set receive_altitude topic
void set_topic_receive_altitude(std::string receive_altitude)
{
    topic_receive_altitude = receive_altitude;
}

// function to set wind_estimation topic
void set_topic_wind_estimation(std::string wind_estimation)
{
    topic_wind_estimation = wind_estimation;
}

// function to set blobDataFlags topic
void set_topic_blobDataFlags(std::string blobDataFlags)
{
    topic_blobDataFlags = blobDataFlags;
}

// function to set final corrected airspeed topic
void set_topic_corrected_airspeed(std::string corrected_airspeed)
{
    topic_corrected_airspeed = corrected_airspeed;
}

// function to receive Desired airspeed (published on to the topic) from the topic itself
void receive_guidance_velocity(const geometry_msgs::TwistStamped::ConstPtr& data)
{
    array_velocity_guidance[0] = data -> twist.linear.x;
    array_velocity_guidance[1] = data -> twist.linear.y;
    array_velocity_guidance[2] = data -> twist.linear.z;
    ROS_INFO("Data received from topic \"/mavros/local_position/velocity\".");
}

// function to receive altitude information (published on to the topic) from the topic itself
void receiveAltitude(const std_msgs::Float64::ConstPtr& data)
{
    altitude = data -> data;
    ROS_INFO("Data received from topic \"/mavros/global_position/rel_alt\".");
}

// function to receive wind related information from the topic itself
void getWindspeed(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& data)
{   // for simulation purpose, gen random values in range -6 to +6
    array_windspeed[0] = data -> twist.twist.linear.x; //rand() % 13 + (-6);
    array_windspeed[1] = data -> twist.twist.linear.y; //rand() % 13 + (-6); 
    array_windspeed[2] = data -> twist.twist.linear.z; //rand() % 13 + (-6);
    ROS_INFO("Data received from topic \"/mavros/wind_estimation\".");
    // but won't show any data since none is being published on it yet so it doesn't show up in callback

    // put covariance values in array or possibly in a 6 x 6 Eigen matrix for further evaluation
    for(int index = 0; index < 36; index++)
    array_covariance[index] = data -> twist.covariance[index]; // figure out it's use
}

// function to receive landing target info from vision controller
void vision_landing_target_info(const pkg_ros_monitor::TargetInfo::ConstPtr& info)
{
    flag_target_detected = info -> detected;
    flag_target_tracking = info -> tracking;
    target_center_x = info -> contour.center.x; // distance x from centre of target
    target_center_y = info -> contour.center.y; // distance y from centre of target
    ROS_INFO("Data received from topic \"/landing_target_info\".");
}

// function to receive velocity commands published by guidance controller for comparison
// and also copy their respective signs for further use
void receiveGroundspeedGuidance(const geometry_msgs::TwistStamped::ConstPtr& data){
    // for simulation purpose, gen random values in range -5 to +5
    array_groundspeed_guidance[0] = data -> twist.linear.x; //rand() % 11 + (-5);
    sign_groundspeed_guidance_vel_x = copysign(1, array_groundspeed_guidance[0]);
    array_groundspeed_guidance[1] = data -> twist.linear.y; //rand() % 11 + (-5);
    sign_groundspeed_guidance_vel_y = copysign(1, array_groundspeed_guidance[1]);
    array_groundspeed_guidance[2] = data -> twist.linear.z; //rand() % 11 + (-5);
    sign_groundspeed_guidance_vel_z = copysign(1, array_groundspeed_guidance[2]);
    ROS_INFO("Data received from topic \"/mavros/setpoint_velocity/cmd_vel_new\".");
}

// function to calculate required predictions and populate "command_geometry_twist" message
void prediction_from_monitor_wind(){

    // make calculations
    
    // APPROACH 
    // we have velocity parameters from vel topic and wind topic
    // calculate relative velocity in all three directions
    // needs to intercept messages from pubCommand publisher from guidance node to compare
    // if values are within +1 and -1, let original messages passthrough
    // if values are bigger, make corrections and publish new values in command_geometry_twist message
    // as long as the UAV is moving in the required direction, we can pass relative or guidance velocity accordingly
    // if direction changes (from +ve to -ve and vice versa), then calculate effective new velocity and publish them
    // also check if new effective velocities <= max possible velocity of the vehicle at that moment in time (using contour.x, contour.y)
    // if calculated effective velocity > max possible velocity, publish only max possible velocity

    // Edit this logic
    // Edit 1 below (Hardcode maximum values)
    max_possible_vel_in_positive_x = 1; // target_center_x / float(20); // publishing timer for topic /landing_target_info_new in application
    max_possible_vel_in_negative_x = -1;
    max_possible_vel_in_positive_y = 1; // target_center_y / float(20); 
    max_possible_vel_in_negative_y = -1;
    max_possible_vel_in_positive_z = 1;
    max_possible_vel_in_negative_z = -1;
    
    for(int index = 0; index < 3; index++){
        // Wind Triangle equation
        array_actual_groundspeed[index] = array_velocity_guidance[index] + array_windspeed[index];
        array_vel_difference[index] = array_groundspeed_guidance[index] - array_actual_groundspeed[index]; 
    } 

    // copy signs of irl velocity parameters for comparison
    sign_actual_groundspeed_vel_x = copysign(1, array_actual_groundspeed[0]);
    sign_actual_groundspeed_vel_y = copysign(1, array_actual_groundspeed[1]);
    sign_actual_groundspeed_vel_z = copysign(1, array_actual_groundspeed[2]);

    // assuming target is already in sight and is being tracked, or it is at least tracked even if visual is lost
    if((flag_target_detected && flag_target_tracking) || (flag_target_tracking)){

        for(int index = 0; index < 3; index++){
            // array_new_airspeed = new_groundspeed - windspeed (Wind Triangle equation)
            array_new_airspeed[index] = array_velocity_guidance[index] - array_windspeed[index];
        }

        // Acquire signs to represent direction of calculated velocities
        sign_new_airspeed_vel_x = copysign(1, array_new_airspeed[0]);
        sign_new_airspeed_vel_y = copysign(1, array_new_airspeed[1]);
        sign_new_airspeed_vel_z = copysign(1, array_new_airspeed[2]);

        if(array_new_airspeed[0] == array_groundspeed_guidance[0]){
            command_geometry_twist.twist.linear.x = array_groundspeed_guidance[0];
        }
        else{
            if((array_new_airspeed[0] <= max_possible_vel_in_positive_x) && (array_new_airspeed[0] >= max_possible_vel_in_negative_x)){    
                    command_geometry_twist.twist.linear.x = array_new_airspeed[0];
                }
                else{
                    command_geometry_twist.twist.linear.x = sign_new_airspeed_vel_x * max_possible_vel_in_positive_x;
                } 
        }

        if(array_new_airspeed[1] == array_groundspeed_guidance[1]){
            command_geometry_twist.twist.linear.y = array_groundspeed_guidance[1];
        }
        else{
            if((array_new_airspeed[1] <= max_possible_vel_in_positive_y) && (array_new_airspeed[1] >= max_possible_vel_in_negative_y)){    
                    command_geometry_twist.twist.linear.y = array_new_airspeed[1];
                }
                else{
                    command_geometry_twist.twist.linear.y = sign_new_airspeed_vel_y * max_possible_vel_in_positive_y;
                }   
        }

        if(array_new_airspeed[2] == array_groundspeed_guidance[2]){
            command_geometry_twist.twist.linear.z = array_groundspeed_guidance[2];
        }
        else{
            if((array_new_airspeed[2] <= max_possible_vel_in_positive_z) && (array_new_airspeed[2] >= max_possible_vel_in_negative_z)){    
                    command_geometry_twist.twist.linear.z = array_new_airspeed[2];
                }
                else{
                    command_geometry_twist.twist.linear.z = sign_new_airspeed_vel_z * max_possible_vel_in_positive_z;
                } 
        }
    } // end of big if
    else{
        // hold the position
        command_geometry_twist.twist.linear.x = 0;
        command_geometry_twist.twist.linear.y = 0;
        command_geometry_twist.twist.linear.z = 0;
    }
} // end of function prediction_from_monitor_wind

// function to publish final command_geometry_twist through the publisher via this monitor
void publish_final_command_wind(){
       
    ROS_INFO("\n\n------------------------------------Data received----------------------------------------\n\n");
    
    ROS_INFO("\n\nDesired airspeed received via topic for parameter x : %f \n""Desired airspeed received via topic for parameter y : %f \n"
    "Desired airspeed received via topic for parameter z : %f \n", array_velocity_guidance[0], array_velocity_guidance[1],
    array_velocity_guidance[2]);

    ROS_INFO("\n\nWind speed received via topic for parameter x : %f \n""Wind speed received via topic for parameter y : %f \n"
    "Wind speed received via topic for parameter z : %f \n", array_windspeed[0], array_windspeed[1],
    array_windspeed[2]);
    
    ROS_INFO("\n\nAltitude Info received from vision controller : %f\n", altitude);

    ROS_INFO("\n\nTarget distance received from vision controller for parameter x : %f \n"
    "Target distance received from vision controller for parameter y : %f \n", target_center_x, target_center_y);

    ROS_INFO("\n\nFlag value for target being detected : %d \n"
    "Flag value for target being tracked : %d \n", flag_target_detected, flag_target_tracking);

    ROS_INFO("\n\nGroundspeed received for comparison from guidance controller for parameter x : %f \n"
    "Groundspeed received for comparison from guidance controller for parameter y : %f \n"
    "Groundspeed received for comparison from guidance controller for parameter z : %f\n",
    array_groundspeed_guidance[0], array_groundspeed_guidance[1], array_groundspeed_guidance[2]);

    // make prediction at set frequency
    prediction_from_monitor_wind();
    
    ROS_INFO("\n\nCorrected airspeed published by monitor for parameter x : %f \n"
    "Corrected airspeed published by monitor for parameter y : %f \n"
    "Corrected airspeed published by monitor for parameter z : %f\n\n",
    command_geometry_twist.twist.linear.x, command_geometry_twist.twist.linear.y, command_geometry_twist.twist.linear.z);
    ROS_INFO("\n\n\n---------------------------------------------------------------------------------------------\n");

    // finally, publish to the topic
    pub_corrected_airspeed.publish(command_geometry_twist);
    ROS_INFO("Data publishing to topic \"/mavros/setpoint_velocity/cmd_vel_new\".");
    ROS_INFO("\n\n------------------------------------End of data block----------------------------------------\n\n");
} // end of function publish_final_command_wind()

///////////////////////// End of monitor_wind code /////////////////////////////////////////////////////////////////////////////////////

///////////////////////// Start of monitor_geo_fence code //////////////////////////////////////////////////////////////////////////////

void monitor_geo_fence_start()
{
    ROS_INFO("start of monitor_geo_fence_start function reached");
    // get single geo fence monitor instance
    //monitor_geo_fence::getInstance();
    //ROS_INFO("monitor_geo_fence::getInstance function executed");
    //monitor_geo_fence::initialize_pub_and_sub();

    // create a nodehandle to enable interaction with ros commands, usually always just after ros::init
    ros::NodeHandle nodeHandle;
    
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and provide it info about our callback function
    dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    callback_variable = boost::bind(&reconfiguration_callback, _1, _2); // same function from the top

    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    
    // create publisher and subscriber objects

    // final publisher to application // topic should just be cmd_vel
    pub_corrected_velocity = nodeHandle.advertise<geometry_msgs::TwistStamped>(topic_corrected_velocity, 1000);
    // publisher to publish new mavros flight state
    pub_new_mavros_state = nodeHandle.advertise<mavros_msgs::State>(topic_new_mavros_state, 1000); 
    // subscriber to receive local position data from controller
    sub_local_position_data = nodeHandle.subscribe(topic_local_position_data, 1000, receive_local_position_data);
    // subscriber to receive velocity commands from the topic itself
    sub_guidance_velocity = nodeHandle.subscribe(topic_guidance_velocity, 1000, receive_guidance_velocity);
    // subscriber to receive current mavros state information
    //sub_current_mavros_state = nodeHandle.subscribe(topic_current_mavros_state, 1000, receive_current_mavros_state);

    //run loop at (10) Hz (always in decimal and faster than what is published through guidance controller)
    ros::Rate loop_rate(10);
    
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
        
        publish_final_command_geo_fence(); // keep calling this function
        
        // sleep for appropriate time to hit mark of (10) Hz
        loop_rate.sleep();
    } // end of while loop
    ROS_INFO("end of monitor_geo_fence_start function reached");
} // end of function monitor_geo_fence_start

// function to get a single instance of geo fence monitor
monitor_geo_fence& monitor_geo_fence::getInstance()
{
    static monitor_geo_fence monitor_geo_fence_instance; // instantiated on first use, guaranteed to be destroyed
    ROS_INFO("monitor_geo_fence::getInstance() function reached\n");
    return monitor_geo_fence_instance;
}

//functions to get pose data in positive and negative directions
void set_max_possible_pose_in_positive_x(int config_max_possible_pose_in_positive_x)
{
    max_possible_pose_in_positive_x = config_max_possible_pose_in_positive_x;
}

void set_max_possible_pose_in_negative_x(int config_max_possible_pose_in_negative_x)
{
    max_possible_pose_in_negative_x = config_max_possible_pose_in_negative_x;
}

void set_max_possible_pose_in_positive_y(int config_max_possible_pose_in_positive_y)
{
    max_possible_pose_in_positive_y = config_max_possible_pose_in_positive_y;
}

void set_max_possible_pose_in_negative_y(int config_max_possible_pose_in_negative_y)
{
    max_possible_pose_in_negative_y = config_max_possible_pose_in_negative_y;
}

void set_max_possible_pose_in_positive_z(int config_max_possible_pose_in_positive_z)
{
    max_possible_pose_in_positive_z = config_max_possible_pose_in_positive_z;
}

void set_max_possible_pose_in_negative_z(int config_max_possible_pose_in_negative_z)
{
    max_possible_pose_in_negative_z = config_max_possible_pose_in_negative_z;
}

void set_critical_radius_from_fence_limit(double critical_radius)
{
    critical_radius_from_fence_limit = critical_radius;
}

void set_radius_of_circle_of_influence_s(double radius_of_circle_of_influence)
{
    radius_of_circle_of_influence_s = radius_of_circle_of_influence;
}

void set_constant_beta(double config_constant_beta)
{
    constant_beta = config_constant_beta;
}

// function to set local position data topic
void set_topic_local_position_data(std::string local_position_data)
{
    topic_local_position_data = local_position_data;
}

// function to set corrected position data topic
void set_topic_corrected_velocity(std::string corrected_velocity)
{
    topic_corrected_velocity = corrected_velocity;
}

// function to set new_mavros_state topic
void set_new_mavros_state(std::string new_mavros_state)
{
    topic_new_mavros_state = new_mavros_state;
}

// function to receive local position data
void receive_local_position_data(const nav_msgs::Odometry::ConstPtr &data)
{
    array_local_position_pose_data[0] = data -> pose.pose.position.x;
    array_local_position_pose_data[1] = data -> pose.pose.position.y;
    array_local_position_pose_data[2] = data -> pose.pose.position.z;
    ROS_INFO("Data received from topic \"mavros/global_position/local\".");
}

// function to receive current mavros state data
/*void receive_current_mavros_state(const mavros_msgs::State::ConstPtr &data)
{
    mode_guided = data -> guided;
    mode_flight = data -> mode;
}*/

// function to calculate required predictions and populate "command_nav_pose" message
void prediction_from_monitor_geo_fence()
{
    /* APPROACH 1
    If the vehicle sways more than the threshold (set in config file) values either side,
    trigger the monitor and publish velocity value 0 (hover at that position).
    */
   
    // get the sign for direction purposes
    //sign_local_position_y = copysign(1, array_local_position_pose_data[1]);

    // since we're not predicting anything for parameters x and z,
    // pass the received velocity values as is
    //command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
    //command_geometry_twist.twist.linear.z = array_velocity_guidance[1];

    // or if we were to :
    // for parameter x :
    /*
    if(array_local_position_pose_data[0] <= max_possible_pose_in_positive_x && array_local_position_pose_data[0] >= max_possible_pose_in_negative_x)
    {
        array_monitor_geo_fence_triggered[0] = "No.";
        
        if(array_monitor_geo_fence_triggered[1] == "Yes." || array_monitor_geo_fence_triggered[2] == "Yes.")
        {
            // keep hovering at that position
            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0; 
            command_geometry_twist.twist.linear.z = 0;
        }
        else
        {
            command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
        } // end of inner if - else
    }
    else
    {
        array_monitor_geo_fence_triggered[0] = "Yes.";
        // keep hovering at that position
        command_geometry_twist.twist.linear.x = 0;
        command_geometry_twist.twist.linear.y = 0; 
        command_geometry_twist.twist.linear.z = 0;
    }

    // for parameter y :
    if(array_local_position_pose_data[1] <= max_possible_pose_in_positive_y && array_local_position_pose_data[1] >= max_possible_pose_in_negative_y)
    {
        array_monitor_geo_fence_triggered[1] = "No.";
        
        if(array_monitor_geo_fence_triggered[0] == "Yes." || array_monitor_geo_fence_triggered[2] == "Yes.")
        {
            // keep hovering at that position
            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0; 
            command_geometry_twist.twist.linear.z = 0;
        }
        else
        {
            command_geometry_twist.twist.linear.y = array_velocity_guidance[1];
        } // end of inner if - else
    }
    else
    {
        array_monitor_geo_fence_triggered[1] = "Yes.";
        // keep hovering at that position
        command_geometry_twist.twist.linear.x = 0;
        command_geometry_twist.twist.linear.y = 0; 
        command_geometry_twist.twist.linear.z = 0;
    }

    // for parameter z :
    if(array_local_position_pose_data[2] <= max_possible_pose_in_positive_z && array_local_position_pose_data[2] >= max_possible_pose_in_negative_z)
    {
        array_monitor_geo_fence_triggered[2] = "No.";
        
        if(array_monitor_geo_fence_triggered[0] == "Yes." || array_monitor_geo_fence_triggered[1] == "Yes.")
        {
            // keep hovering at that position
            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0; 
            command_geometry_twist.twist.linear.z = 0;
        }
        else
        {
            command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
        } // end of inner if - else
    }
    else
    {
        array_monitor_geo_fence_triggered[2] = "Yes.";
        // keep hovering at that position
        command_geometry_twist.twist.linear.x = 0;
        command_geometry_twist.twist.linear.y = 0; 
        command_geometry_twist.twist.linear.z = 0;
    }   
    */
    /*
    APPROACH 2
    Vehicle is by default in "AUTO" mode when in motion.
    Geo fence consists of the boundary point as well as a buffer radius.
    As soon as the vehicle breaches the buffer radius, change the mode to 
    "GUIDED" and bring it within safe zone. When it comes back in safe zone change the
    mode back to "AUTO" so that mission can be resumed from that position.
    Use Potential Fields concept to calculate new velocities.
    */

   // predict which fence limit to consider for calculation (+ve or -ve)
   // get signs of vehicle position direction
   sign_vehicle_pose_x = copysign(1, array_local_position_pose_data[0]);
   sign_vehicle_pose_y = copysign(1, array_local_position_pose_data[1]);
   sign_vehicle_pose_z = copysign(1, array_local_position_pose_data[2]);

   if(sign_vehicle_pose_x == 1)
   {
       fence_limit_to_consider_in_x = max_possible_pose_in_positive_x;
   }
   else
   {
       fence_limit_to_consider_in_x = max_possible_pose_in_negative_x;
   }

   if(sign_vehicle_pose_y == 1)
   {
       fence_limit_to_consider_in_y = max_possible_pose_in_positive_y;
   }
   else
   {
       fence_limit_to_consider_in_y = max_possible_pose_in_negative_y;
   }

   if(sign_vehicle_pose_z == 1)
   {
       fence_limit_to_consider_in_z = max_possible_pose_in_positive_z;
   }
   else
   {
       fence_limit_to_consider_in_z = max_possible_pose_in_negative_z;
   }

   dist_bet_fence_and_vehicle_x = fence_limit_to_consider_in_x - array_local_position_pose_data[0];
   dist_bet_fence_and_vehicle_y = array_local_position_pose_data[1] - fence_limit_to_consider_in_y;
   dist_bet_fence_and_vehicle_z = fence_limit_to_consider_in_z - array_local_position_pose_data[2]; // not used so far

   // calculations for potential field based velocities in two dimensions
    dist_bet_fence_and_vehicle_overall = sqrt((pow(dist_bet_fence_and_vehicle_x, 2.0)) + (pow(dist_bet_fence_and_vehicle_y, 2.0)));
    angle_bet_fence_and_vehicle = atan2((- dist_bet_fence_and_vehicle_y), dist_bet_fence_and_vehicle_x); // in radians

    // actions to be taken
    if(dist_bet_fence_and_vehicle_overall < critical_radius_from_fence_limit)
    {
        //array_monitor_geo_fence_triggered[2] = "Yes.";
        //command_mavros_state.guided = true;
        //command_mavros_state.mode = "GUIDED";
        // keep hovering at that position, delta x and delta y in potential field equations
        command_geometry_twist.twist.linear.x = 0;
        command_geometry_twist.twist.linear.y = 0; 
        command_geometry_twist.twist.linear.z = 0;
    }
    else if ((dist_bet_fence_and_vehicle_overall > critical_radius_from_fence_limit) && 
    (dist_bet_fence_and_vehicle_overall < (critical_radius_from_fence_limit + radius_of_circle_of_influence_s))) 
    {
        //command_mavros_state.guided = true;
        //command_mavros_state.mode = "GUIDED";

        // for direction x
        if((dist_bet_fence_and_vehicle_x > critical_radius_from_fence_limit) && 
        (dist_bet_fence_and_vehicle_x < (critical_radius_from_fence_limit + radius_of_circle_of_influence_s)))
        {
            gradient_x = - constant_beta * (radius_of_circle_of_influence_s + critical_radius_from_fence_limit - dist_bet_fence_and_vehicle_overall)
                        * cos(angle_bet_fence_and_vehicle);
            gradient_y = 0;

            resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
            command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle;
        }  
        // for direction y
        else if((dist_bet_fence_and_vehicle_y > critical_radius_from_fence_limit) && 
        (dist_bet_fence_and_vehicle_y < (critical_radius_from_fence_limit + radius_of_circle_of_influence_s)))
        {
            gradient_x = 0;
            gradient_y = - constant_beta * (radius_of_circle_of_influence_s + critical_radius_from_fence_limit - dist_bet_fence_and_vehicle_overall)
                        * cos(angle_bet_fence_and_vehicle);
 
            resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
            command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle;
        }  
        // for direction z
        command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
        /*
        resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));

        // split resulting velocity into parameters x and y
        command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle * cos(angle_bet_fence_and_vehicle);
        command_geometry_twist.twist.linear.y = resulting_velocity_of_vehicle * sin(angle_bet_fence_and_vehicle);
        */
    }
    else if(dist_bet_fence_and_vehicle_overall > (critical_radius_from_fence_limit + radius_of_circle_of_influence_s))
    {
        //command_mavros_state.guided = false;
        //command_mavros_state.mode = "AUTO";

        command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
        command_geometry_twist.twist.linear.y = array_velocity_guidance[1];
        command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
    }
    
} // end of function prediction_from_monitor_geo_fence()

// function to publish final command_geometry_twist through the publisher via this monitor
void publish_final_command_geo_fence()
{
    ROS_INFO("\n\n------------------------------------Data received----------------------------------------\n\n");

    ROS_INFO("\n\nGeo fence limits set :\n\n"
    "Fence limit set in direction positive x : %f \n"
    "Fence limit set in direction negative x : %f \n"
    "Fence limit set in direction positive y : %f \n"
    "Fence limit set in direction negative y : %f \n"
    "Fence limit set in direction positive z : %f \n"
    "Fence limit set in direction negative z : %f \n"
    "Critical radius from fence limit : %f\n"
    "Radius of circle of influence \"s\" : %f\n",
    max_possible_pose_in_positive_x, max_possible_pose_in_negative_x,
    max_possible_pose_in_positive_y, max_possible_pose_in_negative_y,
    max_possible_pose_in_positive_z, max_possible_pose_in_negative_z,
    critical_radius_from_fence_limit, radius_of_circle_of_influence_s);
    
    ROS_INFO("\n\nLocal position received from 'Home' for parameter x : %f \n"
    "Local position received from \"Home\" for parameter y : %f \n"
    "Local position received from \"Home\" for parameter z : %f \n",
    array_local_position_pose_data[0], array_local_position_pose_data[1], array_local_position_pose_data[2]);

    ROS_INFO("\n\nDesired airspeed received via topic for parameter x : %f \n""Desired airspeed received via topic for parameter y : %f \n"
    "Desired airspeed received via topic for parameter z : %f \n", array_velocity_guidance[0], array_velocity_guidance[1],
    array_velocity_guidance[2]);

    // make prediction at set frequency
    prediction_from_monitor_geo_fence();

    ROS_INFO("\n\n\"GUIDED\" mode : %d\n"
    "Current mode : %s\n"
    "Value of constant beta : %f\n",
    command_mavros_state.guided,
    command_mavros_state.mode.c_str(),
    constant_beta);
    /*
    ROS_INFO("\n\nFence breach in direction x : %s \n"
    "Fence breach in direction y : %s \n"
    "Fence breach in direction z : %s \n",
    array_monitor_geo_fence_triggered[0].c_str(),
    array_monitor_geo_fence_triggered[1].c_str(),
    array_monitor_geo_fence_triggered[2].c_str());
    */
    ROS_INFO("\n\nCorrected airspeed published by monitor for parameter x : %f \n"
    "Corrected airspeed published by monitor for parameter y : %f \n"
    "Corrected airspeed published by monitor for parameter z : %f \n",
    command_geometry_twist.twist.linear.x, command_geometry_twist.twist.linear.y, command_geometry_twist.twist.linear.z);
    ROS_INFO("\n\n\n---------------------------------------------------------------------------------------------\n");
    
    // finally, publish to the topics
    pub_new_mavros_state.publish(command_mavros_state);
    pub_corrected_velocity.publish(command_geometry_twist);
    ROS_INFO("Data publishing to topic \"/mavros/setpoint_velocity/cmd_vel_new\".");
    ROS_INFO("\n\n------------------------------------End of data block----------------------------------------\n\n");
} // end of function publish_final_command_geo_fence()

///////////////////////// End of monitor_geo_fence code /////////////////////////////////////////////////////////////////////////////////////


