/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : node_monitor_geo_fence.cpp
*/

#include "monitor_geo_fence.h"

//-------------------------------------------------------------------------------------------------------------------------------

//<-------------------------------------------Global variables and structures--------------------------------------------------->

std::string topic_guidance_velocity = "";
std::string topic_local_position_data = "";
std::string topic_corrected_velocity = ""; 
std::string topic_current_mavros_state = "";
std::string mode_flight = "";
std::string topic_waypoint_list = "";
std::string topic_home_lat_and_long = "";
std::string topic_global_position_uav = "";

struct waypoint{
    float x_lat;
    float y_long;
    float z_alt;
};

const int array_velocity_guidance_size = 3, array_local_position_pose_data_size = 3, monitor_geo_fence_triggered_size = 3,
array_waypoint_list_size = 100, radius_of_earth = 6371, array_global_position_uav_size = 3;

std::string array_monitor_geo_fence_triggered[monitor_geo_fence_triggered_size] = "No."; // for geo fence monitor

double array_velocity_guidance[array_velocity_guidance_size] = {0}, array_local_position_pose_data[array_local_position_pose_data_size] = {0},
max_possible_pose_in_positive_x = 0.0, max_possible_pose_in_negative_x = 0.0, max_possible_pose_in_positive_y = 0.0, 
max_possible_pose_in_negative_y = 0.0, max_possible_pose_in_positive_z = 0.0, max_possible_pose_in_negative_z = 0.0,
fence_limit_to_consider_in_x, fence_limit_to_consider_in_y, fence_limit_to_consider_in_z,
dist_bet_fence_and_vehicle_x = 0.0, dist_bet_fence_and_vehicle_y = 0.0, dist_bet_fence_and_vehicle_z = 0.0,
critical_radius_from_fence_limit, radius_of_circle_of_influence_s, dist_bet_fence_and_vehicle_overall = 0.0,
angle_bet_fence_and_vehicle = 0.0, gradient_x = 0.0, gradient_y = 0.0, constant_beta = 0.0, resulting_velocity_of_vehicle = 0.0,
resulting_angle_theta = 0.0, critical_radius_start_from_home = 0.0, constant_beta_x = 0.0,constant_beta_y = 0.0,
x_lat_home, y_long_home, x_lat_mission_wp, y_long_mission_wp, diff_in_lat, diff_in_long,
some_parameter_a, some_parameter_c, some_parameter_d, some_parameter_y, some_parameter_x, some_parameter_bearing,
waypoint_current_lat, waypoint_current_long, location_home_lat_x, location_home_long_y, location_home_alt_z, wp_x = 0.0, 
wp_y = 0.0, array_global_position_uav[array_global_position_uav_size] = {0.0};

int sign_vehicle_pose_x, sign_vehicle_pose_y, sign_vehicle_pose_z, size_waypoint_list, waypoint_current;
bool home_init = true, first_conversion = true, list_trigger = true, corrected_auto = false, start_guided = true, flag = true,
monitor_triggered = false;

waypoint array_waypoint_list[array_waypoint_list_size] = {0.0}; // array of struct waypoint

geometry_msgs::TwistStamped command_geometry_twist; // final command_geometry_twist message to be published
nav_msgs::Odometry command_nav_pose; // final command_nav_pose message to be published
mavros_msgs::SetMode command_mavros_set_mode; // final command_mavros_set_mode to be published
mavros_msgs::SetMode command_mavros_set_mode_auto;
mavros_msgs::WaypointSetCurrent command_waypoint_set_current; // final command to publish updated waypoint

// publishers, subscribers and services for monitor_geo_fence
ros::Subscriber sub_guidance_velocity;
ros::Publisher pub_corrected_velocity;
ros::Subscriber sub_local_position_data;
ros::Subscriber sub_current_mavros_state;
ros::ServiceClient srv_mavros_state;
ros::Subscriber sub_waypoint_list;
ros::Subscriber sub_home_lat_and_long;
ros::ServiceClient srv_set_current_waypoint;
ros::Subscriber sub_global_position_uav;

//<----------------------------------------------------------------------------------------------------------------------------->

//<------------------------------------------Local function declarations--------------------------------------------------------->

void publish_final_command_geo_fence();
void get_guidance_controller_velocity(const geometry_msgs::TwistStamped::ConstPtr& data);
void set_topic_guidance_velocity(std::string guidance_velocity);
void set_max_possible_pose_in_positive_x(int config_max_possible_pose_in_positive_x);
void set_max_possible_pose_in_negative_x(int config_max_possible_pose_in_negative_x);
void set_max_possible_pose_in_positive_y(int config_max_possible_pose_in_positive_y);
void set_max_possible_pose_in_negative_y(int config_max_possible_pose_in_negative_y);
void set_max_possible_pose_in_positive_z(int config_max_possible_pose_in_positive_z);
void set_max_possible_pose_in_negative_z(int config_max_possible_pose_in_negative_z);
void set_topic_local_position_data(std::string local_position_data);
void set_topic_corrected_velocity(std::string corrected_velocity);
void set_topic_waypoint_list(std::string waypoint_list);
void set_topic_home_lat_and_long(std::string home_lat_and_long);
void set_topic_global_position_uav(std::string global_position_uav);
void set_constant_beta(double config_constant_beta);
void set_critical_radius_start_from_home(double config_critical_radius_start_from_home);
void set_critical_radius_from_fence_limit(double critical_radius);
void set_radius_of_circle_of_influence_s(double radius_of_circle_of_influence);
void get_local_position_data(const nav_msgs::Odometry::ConstPtr& data);
void get_waypoint_list(const mavros_msgs::WaypointList::ConstPtr& list);
void get_home_lat_and_long(const mavros_msgs::HomePosition::ConstPtr& data);
void convert_lat_long_to_x_y(double x_lat_home, double y_long_home, double x_lat_mission_wp, 
double long_y_mission_wp);
void get_global_position_uav(const sensor_msgs::NavSatFix::ConstPtr& data);
void prediction_from_monitor_geo_fence();
bool uav_in_safe_zone();

//<----------------------------------------------------------------------------------------------------------------------------->

//<------------------------------------------------Initialize monitor node------------------------------------------------------>

int main(int argc, char **argv)
{
    //ROS_INFO("\n\n---------------------------------Welcome--------------------------------------\n\n");

    // initialize ros node with a node name
    ros::init(argc, argv, "node_monitor_geo_fence");
    
    // create monitor object
    monitor_geo_fence obj_monitor_geo_fence;
    // implement class functions
    obj_monitor_geo_fence.init_parameter_server();
    //obj_monitor_geo_fence.dynamic_reconfigure_callback(config, level); // main issue is here, not using same config as before
    obj_monitor_geo_fence.initialize_pub_and_sub();
    obj_monitor_geo_fence.monitor_start();
    //ros::spin();
    return 0;

} // end of initialization

//<------------------------------------------Function definitions---------------------------------------------------------------->

monitor_geo_fence::monitor_geo_fence() : monitor_base(){
    ROS_INFO("monitor_geo_fence constructor called, object initialized\n");
}

void monitor_geo_fence::dynamic_reconfigure_callback(pkg_ros_monitor::monitor_Config &config, uint32_t level){
    
    ROS_INFO("dynamic_reconfigure_callback function reached\n");
    
    // for geo fence monitor
    set_topic_guidance_velocity(config.set_topic_guidance_velocity.c_str());
    set_max_possible_pose_in_positive_x(config.set_max_possible_pose_in_positive_x);
    set_max_possible_pose_in_negative_x(config.set_max_possible_pose_in_negative_x);
    set_max_possible_pose_in_positive_y(config.set_max_possible_pose_in_positive_y);
    set_max_possible_pose_in_negative_y(config.set_max_possible_pose_in_negative_y);
    set_max_possible_pose_in_positive_z(config.set_max_possible_pose_in_positive_z);
    set_max_possible_pose_in_negative_z(config.set_max_possible_pose_in_negative_z);
    set_critical_radius_start_from_home(config.set_critical_radius_start_from_home);
    set_critical_radius_from_fence_limit(config.set_critical_radius_from_fence_limit);
    set_radius_of_circle_of_influence_s(config.set_radius_of_circle_of_influence_s);
    set_constant_beta(config.set_constant_beta);
    set_topic_local_position_data(config.set_topic_local_position_data.c_str());
    set_topic_corrected_velocity(config.set_topic_corrected_velocity.c_str());
    set_topic_waypoint_list(config.set_topic_waypoint_list.c_str());
    set_topic_home_lat_and_long(config.set_topic_home_lat_and_long.c_str());
    set_topic_global_position_uav(config.set_topic_global_position_uav.c_str());

    ROS_INFO("Current geo fence configuration parameters: \n\n"
    "Guidance controller topic to get velocity parameters for comparison from : %s\n\n"
    "Fence limit set in positive x : %f\n"
    "Fence limit set in negative x : %f\n"
    "Fence limit set in positive y : %f\n"
    "Fence limit set in negative y : %f\n"
    "Fence limit set in positive z : %f\n"
    "Fence limit set in negative z : %f\n\n"
    "Critical radius (calculating from home) : %f\n"
    "Critical radius from fence limit : %f\n"
    "Radius of circle of influence \"s\" : %f\n\n"
    "Value for constant beta (for potential field calculation) : %f\n\n"
    "Topic to get local position data from : %s\n"
    "Topic to get desired velocity parameters from : %s\n"
    "Topic to publish corrected velocity to : %s \n"
    "Topic to get waypoint list from : %s\n"
    "Topic to get home location from : %s\n"
    "Topic name to get global position of UAV in lat and long : %s\n\n",
    config.set_topic_guidance_velocity.c_str(),
    config.set_max_possible_pose_in_positive_x, config.set_max_possible_pose_in_negative_x,
    config.set_max_possible_pose_in_positive_y, config.set_max_possible_pose_in_negative_y,
    config.set_max_possible_pose_in_positive_z, config.set_max_possible_pose_in_negative_z,
    config.set_critical_radius_start_from_home,
    config.set_critical_radius_from_fence_limit, config.set_radius_of_circle_of_influence_s,
    config.set_constant_beta,
    config.set_topic_local_position_data.c_str(),
    config.set_topic_desired_airspeed.c_str(),
    config.set_topic_corrected_velocity.c_str(), config.set_topic_waypoint_list.c_str(),
    config.set_topic_home_lat_and_long.c_str(), config.set_topic_global_position_uav.c_str());

    ROS_INFO("dynamic_reconfigure_callback function ended\n");
}

void monitor_geo_fence::initialize_pub_and_sub(){
    
    ROS_INFO("start of initialize pub and sub function reached\n");
    // create a nodehandle to enable interaction with ros commands, usually always just after ros::init
    ros::NodeHandle nodeHandle;

    // subscriber to receive home location coordinates
    //sub_home_lat_and_long = nodeHandle.subscribe(topic_home_lat_and_long, 1000, get_home_lat_and_long);
    // final publisher to application // topic should just be cmd_vel
    pub_corrected_velocity = nodeHandle.advertise<geometry_msgs::TwistStamped>(topic_corrected_velocity, 1000);
    // publisher to publish new mavros flight state
    //pub_new_mavros_state = nodeHandle.advertise<mavros_msgs::State>(topic_new_mavros_state, 1000); 
    // subscriber to receive local position data from controller
    //sub_local_position_data = nodeHandle.subscribe(topic_local_position_data, 1000, get_local_position_data);
    // subscriber to receive velocity commands from the topic itself
    sub_guidance_velocity = nodeHandle.subscribe(topic_guidance_velocity, 1000, get_guidance_controller_velocity);
    // subscriber to get mission waypoint list
    sub_waypoint_list = nodeHandle.subscribe(topic_waypoint_list, 1000, get_waypoint_list);
    // subscriber to get global UAV position and use it as home location
    sub_global_position_uav = nodeHandle.subscribe(topic_global_position_uav, 1000, get_global_position_uav);
    // service to set mode of the vehicle
    srv_mavros_state = nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    // service to set updated current waypoint
    srv_set_current_waypoint = nodeHandle.serviceClient<mavros_msgs::WaypointSetCurrent>("/mavros/mission/set_current");

    ROS_INFO("end of initialize pub and sub function reached\n");
}

void monitor_geo_fence::monitor_start(){

    ROS_INFO("start of monitor_start function reached");
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
        
        publish_final_command_geo_fence(); // keep calling this function
        
        // sleep for appropriate time to hit mark of (10) Hz
        loop_rate.sleep();
    } // end of while loop
    ROS_INFO("end of monitor_start function reached");
}

// function to set guidance velocity topic
void set_topic_guidance_velocity(std::string guidance_velocity)
{
    topic_guidance_velocity = guidance_velocity;
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

void set_critical_radius_start_from_home(double config_critical_radius_start_from_home)
{
    critical_radius_start_from_home = config_critical_radius_start_from_home;
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

// function to set waypoint_list topic
void set_topic_waypoint_list(std::string waypoint_list)
{
    topic_waypoint_list = waypoint_list;
}

// function to set home location topic name
void set_topic_home_lat_and_long(std::string home_lat_and_long)
{
    topic_home_lat_and_long = home_lat_and_long;
}

// function to get global UAV position, then use it's very first position and set
// it as home location
void set_topic_global_position_uav(std::string global_position_uav)
{
    topic_global_position_uav = global_position_uav;
}

// function to receive Desired airspeed (published on to the topic) from the topic itself
void get_guidance_controller_velocity(const geometry_msgs::TwistStamped::ConstPtr& data)
{
    array_velocity_guidance[0] = data -> twist.linear.x;
    array_velocity_guidance[1] = data -> twist.linear.y;
    array_velocity_guidance[2] = data -> twist.linear.z;
    ROS_INFO("Data received from topic \"/mavros/local_position/velocity\".");
}
/*
// function to receive local position data
void get_local_position_data(const nav_msgs::Odometry::ConstPtr &data)
{
    array_local_position_pose_data[0] = data -> pose.pose.position.x;
    array_local_position_pose_data[1] = data -> pose.pose.position.y;
    array_local_position_pose_data[2] = data -> pose.pose.position.z;
    ROS_INFO("Data received from topic \"mavros/global_position/local\".");
}
*/
// function to receive global uav position, and also use it to set home location in lat and long
void get_global_position_uav(const sensor_msgs::NavSatFix::ConstPtr& data)
{
    if(home_init){
        location_home_lat_x = data -> latitude;
        location_home_long_y = data -> longitude;
        location_home_alt_z = data -> altitude;
        home_init = false;
    }

    array_global_position_uav[0] = data -> latitude;
    array_global_position_uav[1] = data -> longitude;
    array_global_position_uav[2] = data -> altitude;

    // convert current waypoint's lat long coordinates to x y coordinates
    convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_global_position_uav[0],
    array_global_position_uav[1]);
    // assuming home location to be 0, 0

    array_local_position_pose_data[0] = wp_x;
    array_local_position_pose_data[1] = wp_y;
    //array_local_position_pose_data[2] = data -> altitude;

    ROS_INFO("Data received from topic \"mavros/global_position/global\".\n");
    ROS_INFO("\nlocation_home_lat_x : %f\n"
    "location_home_long_y : %f\n"
    "location_home_alt_z : %f\n"
    "UAV lat from global x : %f\n"
    "UAV long from global y : %f\n"
    "UAV position x : %f\n"
    "UAV position y : %f\n\n", 
    location_home_lat_x, location_home_long_y, location_home_alt_z, 
    array_global_position_uav[0], array_global_position_uav[1],
    array_local_position_pose_data[0], array_local_position_pose_data[1]);
}

// function to receive mission waypoints
void get_waypoint_list(const mavros_msgs::WaypointList::ConstPtr& list)
{
    waypoint_current = list -> current_seq;

    if(list_trigger){
        size_waypoint_list = list -> waypoints.size();

        for(int i = 0; i < size_waypoint_list; i++){
            array_waypoint_list[i].x_lat = list -> waypoints[i].x_lat;
            array_waypoint_list[i].y_long = list -> waypoints[i].y_long;
            array_waypoint_list[i].z_alt = list -> waypoints[i].z_alt;
        }
        list_trigger = false;
    }
    // convert current waypoint's lat long coordinates to x y coordinates
    convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
    array_waypoint_list[waypoint_current].y_long);
    // assuming home location to be 0, 0
    ROS_INFO("Data received from topic \"/mavros/mission/waypoints\".");
    ROS_INFO("No. of waypoints received : %d\n"
    "Current waypoint : %d\n\n"
    "wp_x : %f\n""wp_y : %f\n",
    size_waypoint_list, waypoint_current, wp_x, wp_y);
}
/*
// function to get home location's lat and long
void get_home_lat_and_long(const mavros_msgs::HomePosition::ConstPtr& data)
{
    location_home_lat_x = data -> geo.latitude;
    location_home_long_y = data -> geo.longitude;
    location_home_alt_z = data -> geo.altitude;
    //ROS_INFO("%f, %f, %f", location_home_lat_x, location_home_long_y, location_home_alt_z);
}
*/
// function to convert lat long coordinates of a waypoint to simple x y coordinates
void convert_lat_long_to_x_y(double x_lat_home, double y_long_home, double x_lat_mission_wp, 
double y_long_mission_wp)
{
    diff_in_lat = x_lat_mission_wp - x_lat_home;
    diff_in_long = y_long_mission_wp - y_long_home;

    some_parameter_a = (sin(diff_in_lat / 2.0) * sin(diff_in_lat / 2.0)) + (cos(x_lat_home) * cos(x_lat_mission_wp) * 
                        (sin(diff_in_long / 2.0) * sin(diff_in_long / 2.0)));
    some_parameter_c = 2 * atan2(sqrt(some_parameter_a), sqrt(1 - some_parameter_a));
    some_parameter_d = radius_of_earth * some_parameter_c * 1000;
    some_parameter_y = sin(diff_in_long) * cos(x_lat_home);
    some_parameter_x = (sin(x_lat_home) * cos(x_lat_mission_wp) * cos(diff_in_long)) - 
                        (cos(x_lat_home) * sin(x_lat_mission_wp));
    some_parameter_x = - some_parameter_x;
    some_parameter_bearing = fmod((atan2(some_parameter_y, some_parameter_x) + (2 * M_PI)), (2 * M_PI));

    wp_x = (some_parameter_d * cos(some_parameter_bearing));
    wp_y = (some_parameter_d * sin(some_parameter_bearing));
}

// check if UAV is in or out of critical radius
bool uav_in_safe_zone()
{   // if UAV is out of critical radius, return true
    if((abs(array_local_position_pose_data[0]) < (abs(fence_limit_to_consider_in_x) - critical_radius_from_fence_limit)) &&
    (abs(array_local_position_pose_data[1]) < (abs(fence_limit_to_consider_in_y) - critical_radius_from_fence_limit))){
        return true;
    }else{
        return false;
    }
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

    // since we're not predicting anything in directions x and z,
    // pass the received velocity values as is
    //command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
    //command_geometry_twist.twist.linear.z = array_velocity_guidance[1];

    // or if we were to :
    /*
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

    // in direction x :
    if(array_local_position_pose_data[0] <= critical_radius_start_from_home && 
    array_local_position_pose_data[0] >= (-1 * critical_radius_start_from_home))
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

        if(abs(array_local_position_pose_data[0]) < abs((abs(fence_limit_to_consider_in_x) + critical_radius_start_from_home) / 2))
        {
            command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
        }
        else
        {
            // keep hovering at that position
            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0; 
            command_geometry_twist.twist.linear.z = 0;
        } // end of inner if - else
    } // end of outer if - else

    // in direction y :
    if(array_local_position_pose_data[1] <= critical_radius_start_from_home && 
    array_local_position_pose_data[1] >= (-1 * critical_radius_start_from_home))
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

        if(abs(array_local_position_pose_data[1]) < abs((abs(fence_limit_to_consider_in_y) + critical_radius_start_from_home) / 2))
        {
            command_geometry_twist.twist.linear.x = array_velocity_guidance[1];
        }
        else
        {
            // keep hovering at that position
            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0; 
            command_geometry_twist.twist.linear.z = 0;
        } // end of inner if - else
    } // end of outer if - else

    // in direction z :
    if(array_local_position_pose_data[2] <= critical_radius_start_from_home && 
    array_local_position_pose_data[2] >= (-1 * critical_radius_start_from_home))
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

        if(abs(array_local_position_pose_data[2]) < abs((abs(fence_limit_to_consider_in_z) + critical_radius_start_from_home) / 2))
        {
            command_geometry_twist.twist.linear.x = array_velocity_guidance[2];
        }
        else
        {
            // keep hovering at that position
            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0; 
            command_geometry_twist.twist.linear.z = 0;
        } // end of inner if - else
    } // end of outer if - else   
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
   /*
   // predict which fence limit to consider for calculation (+ve or -ve)
   // get signs of vehicle position direction
   sign_vehicle_pose_x = copysign(1, array_local_position_pose_data[0]);
   sign_vehicle_pose_y = copysign(1, array_local_position_pose_data[1]);
   sign_vehicle_pose_z = copysign(1, array_local_position_pose_data[2]);

   if(sign_vehicle_pose_x == 1)
   {
       fence_limit_to_consider_in_x = max_possible_pose_in_positive_x;
       constant_beta_x = constant_beta * -1;
   }
   else
   {
       fence_limit_to_consider_in_x = max_possible_pose_in_negative_x;
       constant_beta_x = constant_beta * 1;
   }

   if(sign_vehicle_pose_y == 1)
   {
       fence_limit_to_consider_in_y = max_possible_pose_in_positive_y;
       constant_beta_y = constant_beta * -1;
   }
   else
   {
       fence_limit_to_consider_in_y = max_possible_pose_in_negative_y;
       constant_beta_y = constant_beta * 1;
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
   dist_bet_fence_and_vehicle_y = fence_limit_to_consider_in_y - array_local_position_pose_data[1];
   dist_bet_fence_and_vehicle_z = fence_limit_to_consider_in_z - array_local_position_pose_data[2]; // not used so far

   // calculations for potential field based velocities in two dimensions
    //dist_bet_fence_and_vehicle_overall = sqrt((pow(dist_bet_fence_and_vehicle_x, 2.0)) + (pow(dist_bet_fence_and_vehicle_y, 2.0))); //wrong


    // actions to be taken
    // in direction x : 
    if(abs(array_local_position_pose_data[0]) < (abs(fence_limit_to_consider_in_x) - critical_radius_from_fence_limit))
    {
        array_monitor_geo_fence_triggered[0] = "No.";
        command_mavros_set_mode.request.base_mode = 220; // mode : AUTO ARMED
        command_mavros_set_mode.request.custom_mode = "AUTO";

        command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
    }
    else if ((abs(array_local_position_pose_data[0]) > (abs(fence_limit_to_consider_in_x) - critical_radius_from_fence_limit)) && 
    (abs(array_local_position_pose_data[0]) < abs(fence_limit_to_consider_in_x))) 
    {
        array_monitor_geo_fence_triggered[0] = "Yes.";
        command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
        command_mavros_set_mode.request.custom_mode = "GUIDED";        
        
        gradient_x = constant_beta_x;
        gradient_y = 0;

        angle_bet_fence_and_vehicle = atan2(gradient_y, gradient_x); // in radians

        resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
        command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle * cos(angle_bet_fence_and_vehicle);
    }        
    else if(abs(array_local_position_pose_data[0]) > abs(fence_limit_to_consider_in_x))
    {
        array_monitor_geo_fence_triggered[0] = "Yes.";
        command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
        command_mavros_set_mode.request.custom_mode = "GUIDED";        

        command_geometry_twist.twist.linear.x = 0;
    }  

    // in direction y :
    if(abs(array_local_position_pose_data[1]) < (abs(fence_limit_to_consider_in_y) - critical_radius_from_fence_limit))
    {
        array_monitor_geo_fence_triggered[1] = "No.";
        command_mavros_set_mode.request.base_mode = 220; // mode : AUTO ARMED
        command_mavros_set_mode.request.custom_mode = "AUTO";

        command_geometry_twist.twist.linear.y = array_velocity_guidance[1];
    }
    else if ((abs(array_local_position_pose_data[1]) > (abs(fence_limit_to_consider_in_y) - critical_radius_from_fence_limit)) && 
    (abs(array_local_position_pose_data[1]) < abs(fence_limit_to_consider_in_y))) 
    {
        array_monitor_geo_fence_triggered[1] = "Yes.";
        command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
        command_mavros_set_mode.request.custom_mode = "GUIDED";        
        
        gradient_x = 0;
        gradient_y = constant_beta_y;

        angle_bet_fence_and_vehicle = atan2(gradient_y, gradient_x); // in radians

        resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
        command_geometry_twist.twist.linear.y = resulting_velocity_of_vehicle * sin(angle_bet_fence_and_vehicle);
    }        
    else if(abs(array_local_position_pose_data[1]) > abs(fence_limit_to_consider_in_y))
    {
        array_monitor_geo_fence_triggered[1] = "Yes.";
        command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
        command_mavros_set_mode.request.custom_mode = "GUIDED";        

        command_geometry_twist.twist.linear.y = 0;
    }  
    
    // for direction z
    command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
    */
    /*
    if(dist_bet_fence_and_vehicle_overall < critical_radius_from_fence_limit)
    {
        //array_monitor_geo_fence_triggered[2] = "Yes.";
        command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
        command_mavros_set_mode.request.custom_mode = "GUIDED";

        // keep hovering at that position, delta x and delta y in potential field equations
        command_geometry_twist.twist.linear.x = 0;
        command_geometry_twist.twist.linear.y = 0; 
        command_geometry_twist.twist.linear.z = 0;
    }
    else if ((dist_bet_fence_and_vehicle_overall > critical_radius_from_fence_limit) && 
    (dist_bet_fence_and_vehicle_overall < (critical_radius_from_fence_limit + radius_of_circle_of_influence_s))) 
    {
        command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
        command_mavros_set_mode.request.custom_mode = "GUIDED";
        /*
        gradient_x = - constant_beta * (radius_of_circle_of_influence_s + critical_radius_from_fence_limit - dist_bet_fence_and_vehicle_overall)
                        * cos(angle_bet_fence_and_vehicle);
        gradient_y = - constant_beta * (radius_of_circle_of_influence_s + critical_radius_from_fence_limit - dist_bet_fence_and_vehicle_overall)
                        * sin(angle_bet_fence_and_vehicle);;

        resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
        resulting_angle_theta = atan2(gradient_y, gradient_x);
        command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle * cos(resulting_angle_theta);
        command_geometry_twist.twist.linear.y = resulting_velocity_of_vehicle * sin(resulting_angle_theta);
        
        
        // for direction x
        if((dist_bet_fence_and_vehicle_x > critical_radius_from_fence_limit) && 
        (dist_bet_fence_and_vehicle_x < (critical_radius_from_fence_limit + radius_of_circle_of_influence_s)))
        {
            //gradient_x = - constant_beta * (radius_of_circle_of_influence_s + critical_radius_from_fence_limit - dist_bet_fence_and_vehicle_overall)
            //            * cos(angle_bet_fence_and_vehicle);
            gradient_x = - constant_beta;
            gradient_y = 0;

            resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
            command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle * cos(angle_bet_fence_and_vehicle);
        }  
        // for direction y
        if((dist_bet_fence_and_vehicle_y > critical_radius_from_fence_limit) && 
        (dist_bet_fence_and_vehicle_y < (critical_radius_from_fence_limit + radius_of_circle_of_influence_s)))
        {
            gradient_x = 0;
            gradient_y = - constant_beta;
            //gradient_y = - constant_beta * (radius_of_circle_of_influence_s + critical_radius_from_fence_limit - dist_bet_fence_and_vehicle_overall)
            //            * cos(angle_bet_fence_and_vehicle);
 
            resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
            command_geometry_twist.twist.linear.y = resulting_velocity_of_vehicle * sin(angle_bet_fence_and_vehicle);
        }  
        
        // for direction z
        command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
        /*
        resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));

        // split resulting velocity into parameters x and y
        command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle * cos(angle_bet_fence_and_vehicle);
        command_geometry_twist.twist.linear.y = resulting_velocity_of_vehicle * sin(angle_bet_fence_and_vehicle);
        
    }
    else if(dist_bet_fence_and_vehicle_overall > (critical_radius_from_fence_limit + radius_of_circle_of_influence_s))
    {
        command_mavros_set_mode.request.base_mode = 220; // mode : AUTO ARMED
        command_mavros_set_mode.request.custom_mode = "AUTO";

        command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
        command_geometry_twist.twist.linear.y = array_velocity_guidance[1];
        command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
    }  
     */ 

    /*
    APPROACH 3 : 
    Get waypoint list from the autopilot
    Mission is started in mode AUTO
    Check if the current waypoint is out of fence limit or not (using haversine conversion formula)
    If fence is breached / or vehicle enters potential field region and is about to breach fence, 
    skip the current waypoint and call a service to set the next waypoint as the current waypoint.
    Keep flight mode AUTO unless the fence is completely breached, in which case switch it to GUIDED
    */
   
   // predict which fence limit to consider for calculation (+ve or -ve)
   // get signs of vehicle position direction
   sign_vehicle_pose_x = copysign(1, array_local_position_pose_data[0]);
   sign_vehicle_pose_y = copysign(1, array_local_position_pose_data[1]);
   sign_vehicle_pose_z = copysign(1, array_local_position_pose_data[2]);

   if(sign_vehicle_pose_x == 1)
   {
       fence_limit_to_consider_in_x = max_possible_pose_in_positive_x;
       constant_beta_x = constant_beta * -1;
   }
   else
   {
       fence_limit_to_consider_in_x = max_possible_pose_in_negative_x;
       constant_beta_x = constant_beta * 1;
   }

   if(sign_vehicle_pose_y == 1)
   {
       fence_limit_to_consider_in_y = max_possible_pose_in_positive_y;
       constant_beta_y = constant_beta * -1;
   }
   else
   {
       fence_limit_to_consider_in_y = max_possible_pose_in_negative_y;
       constant_beta_y = constant_beta * 1;
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
   dist_bet_fence_and_vehicle_y = fence_limit_to_consider_in_y - array_local_position_pose_data[1];
   dist_bet_fence_and_vehicle_z = fence_limit_to_consider_in_z - array_local_position_pose_data[2]; // not used so far

   // calculations for potential field based velocities in two dimensions
    //dist_bet_fence_and_vehicle_overall = sqrt((pow(dist_bet_fence_and_vehicle_x, 2.0)) + (pow(dist_bet_fence_and_vehicle_y, 2.0))); //wrong
    /*ROS_INFO(//"\n\nwp_mission_x : %f\n""wp_y : %f\n"
    "lat_home_x %f\n"
    "long_home_y %f\n"
    "waypoint_lat_x %f\n"
    "waypoint_long_y %f\n\n", //wp_x, wp_y, 
    location_home_lat_x, location_home_long_y, 
    array_waypoint_list[waypoint_current].x_lat,
    array_waypoint_list[waypoint_current].y_long); */
    /*
    if(first_conversion){
        // convert current waypoint's lat long coordinates to x y coordinates
        convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
        array_waypoint_list[waypoint_current].y_long);
        // assuming home location to be 0, 0
        first_conversion = false;
    }
    */
    // actions to be taken
    // in direction x : 
    
    if(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
        ROS_INFO("Service waypoint skip called\n");
        ROS_INFO("Old waypoint index : %d\n", waypoint_current);
        ROS_INFO("Fence breach at x : %f y : %f\n", wp_x, wp_y);

        while(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
            // convert current waypoint's lat long coordinates to x y coordinates
            convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
            array_waypoint_list[waypoint_current].y_long);
            // assuming home location to be 0, 0
            if(abs(wp_x) < abs(fence_limit_to_consider_in_x) && abs(wp_y) < abs(fence_limit_to_consider_in_y)){
                break; // do nothing
            }
            else{
                waypoint_current = waypoint_current + 1;
            }
            break;
        }

        command_waypoint_set_current.request.wp_seq = waypoint_current;
        ROS_INFO("New waypoint index : %d\n", command_waypoint_set_current.request.wp_seq);
        
        if(srv_set_current_waypoint.call(command_waypoint_set_current)){
            ROS_INFO("Waypoint service called successfully\n");
        } 
        else {
            ROS_INFO("Service call failed\n");
        }
        
        ROS_INFO("Waypoint updation success : %d\n", command_waypoint_set_current.response.success);
    } 
    /*
   if(monitor_triggered == false)
   {
       // in direction x :

       if(abs(array_local_position_pose_data[0]) < (abs(fence_limit_to_consider_in_x) - critical_radius_from_fence_limit))
        {
            array_monitor_geo_fence_triggered[0] = "No.";
            command_mavros_set_mode.request.base_mode = 220; // mode : AUTO ARMED
            command_mavros_set_mode.request.custom_mode = "AUTO";

            command_geometry_twist.twist.linear.x = array_velocity_guidance[0];
        }
        else if ((abs(array_local_position_pose_data[0]) > (abs(fence_limit_to_consider_in_x) - critical_radius_from_fence_limit)) && 
        (abs(array_local_position_pose_data[0]) < abs(fence_limit_to_consider_in_x))) 
        {
            array_monitor_geo_fence_triggered[0] = "Yes.";
            command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
            command_mavros_set_mode.request.custom_mode = "GUIDED";        
            
            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }

            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0;
            command_geometry_twist.twist.linear.z = 0;

            if(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                ROS_INFO("Service waypoint skip called\n");
                ROS_INFO("Old waypoint index : %d\n", waypoint_current);
                ROS_INFO("Fence breach at x : %f y : %f\n", wp_x, wp_y);

                while(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                    // convert current waypoint's lat long coordinates to x y coordinates
                    convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
                    array_waypoint_list[waypoint_current].y_long);
                    // assuming home location to be 0, 0
                    if(abs(wp_x) < abs(fence_limit_to_consider_in_x) && abs(wp_y) < abs(fence_limit_to_consider_in_y)){
                        break; // do nothing
                    }
                    else{
                        waypoint_current = waypoint_current + 1;
                    }
                    break;
                } // end of inner while

                command_waypoint_set_current.request.wp_seq = waypoint_current;
                ROS_INFO("New waypoint index : %d\n", command_waypoint_set_current.request.wp_seq);
                
                if(srv_set_current_waypoint.call(command_waypoint_set_current)){
                    ROS_INFO("Waypoint service called successfully\n");
                } 
                else {
                    ROS_INFO("Service call failed\n");
                }
                
                ROS_INFO("Waypoint updation success : %d\n", command_waypoint_set_current.response.success);
            } // end of outer if

            array_monitor_geo_fence_triggered[0] = "Yes.";
            command_mavros_set_mode_auto.request.base_mode = 220; // mode : AUTO ARMED
            command_mavros_set_mode_auto.request.custom_mode = "AUTO";  

            if(srv_mavros_state.call(command_mavros_set_mode_auto)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode_auto.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            } 
            monitor_triggered = true;
            
            gradient_x = constant_beta_x;
            gradient_y = 0;

            angle_bet_fence_and_vehicle = atan2(gradient_y, gradient_x); // in radians

            resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
            command_geometry_twist.twist.linear.x = resulting_velocity_of_vehicle * cos(angle_bet_fence_and_vehicle);
        }        
        else if(abs(array_local_position_pose_data[0]) > abs(fence_limit_to_consider_in_x))
        { // checking if in case the potential field is not strong enough to stop the vehicle from breaching the fence
            array_monitor_geo_fence_triggered[0] = "Yes.";
            command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
            command_mavros_set_mode.request.custom_mode = "GUIDED";        
            
            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }

            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0;
            command_geometry_twist.twist.linear.z = 0;
            
            if(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                ROS_INFO("Service waypoint skip called\n");
                ROS_INFO("Old waypoint index : %d\n", waypoint_current);
                ROS_INFO("Fence breach at x : %f y : %f\n", wp_x, wp_y);

                while(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                    // convert current waypoint's lat long coordinates to x y coordinates
                    convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
                    array_waypoint_list[waypoint_current].y_long);
                    // assuming home location to be 0, 0
                    if(abs(wp_x) < abs(fence_limit_to_consider_in_x) && abs(wp_y) < abs(fence_limit_to_consider_in_y)){
                        break; // do nothing
                    }
                    else{
                        waypoint_current = waypoint_current + 1;
                    }
                    break;
                } // end of inner while

                command_waypoint_set_current.request.wp_seq = waypoint_current;
                ROS_INFO("New waypoint index : %d\n", command_waypoint_set_current.request.wp_seq);
                
                if(srv_set_current_waypoint.call(command_waypoint_set_current)){
                    ROS_INFO("Waypoint service called successfully\n");
                } 
                else {
                    ROS_INFO("Service call failed\n");
                }
                
                ROS_INFO("Waypoint updation success : %d\n", command_waypoint_set_current.response.success);
            } // end of inner if

            array_monitor_geo_fence_triggered[0] = "Yes.";
            command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
            command_mavros_set_mode.request.custom_mode = "GUIDED";  

            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }     
            monitor_triggered = true; 

            command_geometry_twist.twist.linear.x = 0;
        }  
        
        // in direction y :
        
        if(abs(array_local_position_pose_data[1]) < (abs(fence_limit_to_consider_in_y) - critical_radius_from_fence_limit))
        {
            array_monitor_geo_fence_triggered[1] = "No.";
            command_mavros_set_mode.request.base_mode = 220; // mode : AUTO ARMED
            command_mavros_set_mode.request.custom_mode = "AUTO";

            command_geometry_twist.twist.linear.y = array_velocity_guidance[1];
        }
        else if ((abs(array_local_position_pose_data[1]) > (abs(fence_limit_to_consider_in_y) - critical_radius_from_fence_limit)) && 
        (abs(array_local_position_pose_data[1]) < abs(fence_limit_to_consider_in_y))) 
        {
            array_monitor_geo_fence_triggered[0] = "Yes.";
            command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
            command_mavros_set_mode.request.custom_mode = "GUIDED";        
            
            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }

            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0;
            command_geometry_twist.twist.linear.z = 0;

            if(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                ROS_INFO("Service waypoint skip called\n");
                ROS_INFO("Old waypoint index : %d\n", waypoint_current);
                ROS_INFO("Fence breach at x : %f y : %f\n", wp_x, wp_y);

                while(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                    // convert current waypoint's lat long coordinates to x y coordinates
                    convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
                    array_waypoint_list[waypoint_current].y_long);
                    // assuming home location to be 0, 0
                    if(abs(wp_x) < abs(fence_limit_to_consider_in_x) && abs(wp_y) < abs(fence_limit_to_consider_in_y)){
                        break; // do nothing
                    }
                    else{
                        waypoint_current = waypoint_current + 1;
                    }
                    break;
                } // end of inner while

                command_waypoint_set_current.request.wp_seq = waypoint_current;
                ROS_INFO("New waypoint index : %d\n", command_waypoint_set_current.request.wp_seq);
                
                if(srv_set_current_waypoint.call(command_waypoint_set_current)){
                    ROS_INFO("Waypoint service called successfully\n");
                } 
                else {
                    ROS_INFO("Service call failed\n");
                }
                
                ROS_INFO("Waypoint updation success : %d\n", command_waypoint_set_current.response.success);
            } // end of inner if

            array_monitor_geo_fence_triggered[1] = "Yes.";
            command_mavros_set_mode.request.base_mode = 220; // mode : AUTO ARMED
            command_mavros_set_mode.request.custom_mode = "AUTO";    

            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }  
            monitor_triggered = true;  
            
            gradient_x = 0;
            gradient_y = constant_beta_y;

            angle_bet_fence_and_vehicle = atan2(gradient_y, gradient_x); // in radians

            resulting_velocity_of_vehicle = sqrt(pow(gradient_x, 2.0) + pow(gradient_y, 2.0));
            command_geometry_twist.twist.linear.y = resulting_velocity_of_vehicle * sin(angle_bet_fence_and_vehicle);
        }        
        else if(abs(array_local_position_pose_data[1]) > abs(fence_limit_to_consider_in_y))
        { // checking if in case the potential field is not strong enough to stop the vehicle from breaching the fence
            array_monitor_geo_fence_triggered[0] = "Yes.";
            command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
            command_mavros_set_mode.request.custom_mode = "GUIDED";        
            
            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }

            command_geometry_twist.twist.linear.x = 0;
            command_geometry_twist.twist.linear.y = 0;
            command_geometry_twist.twist.linear.z = 0;
            
            if(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                ROS_INFO("Service waypoint skip called\n");
                ROS_INFO("Old waypoint index : %d\n", waypoint_current);
                ROS_INFO("Fence breach at x : %f y : %f\n", wp_x, wp_y);

                while(abs(wp_x) > abs(fence_limit_to_consider_in_x) || abs(wp_y) > abs(fence_limit_to_consider_in_y)){
                    // convert current waypoint's lat long coordinates to x y coordinates
                    convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
                    array_waypoint_list[waypoint_current].y_long);
                    // assuming home location to be 0, 0
                    if(abs(wp_x) < abs(fence_limit_to_consider_in_x) && abs(wp_y) < abs(fence_limit_to_consider_in_y)){
                        break; // do nothing
                    }
                    else{
                        waypoint_current = waypoint_current + 1;
                    }
                    break;
                } // end of inner while

                command_waypoint_set_current.request.wp_seq = waypoint_current;
                ROS_INFO("New waypoint index : %d\n", command_waypoint_set_current.request.wp_seq);
                
                if(srv_set_current_waypoint.call(command_waypoint_set_current)){
                    ROS_INFO("Waypoint service called successfully\n");
                } 
                else {
                    ROS_INFO("Service call failed\n");
                }
                
                ROS_INFO("Waypoint updation success : %d\n", command_waypoint_set_current.response.success);
            } // end of inner if

            array_monitor_geo_fence_triggered[1] = "Yes.";
            command_mavros_set_mode.request.base_mode = 216; // mode : GUIDED ARMED
            command_mavros_set_mode.request.custom_mode = "GUIDED";   

            if(srv_mavros_state.call(command_mavros_set_mode)){
                ROS_INFO("Service set mode called successfully\n");
                ROS_INFO("New flight mode set : %d\n\n", command_mavros_set_mode.response.mode_sent);
            }
            else{
                ROS_INFO("Service call failed\n");
            }     
            monitor_triggered = true;

            command_geometry_twist.twist.linear.y = 0;
        }  

   } // end of if monitor_triggered

   if(uav_in_safe_zone() && monitor_triggered)
   {
       monitor_triggered = false;
   }
    */
   // for direction z
    command_geometry_twist.twist.linear.z = array_velocity_guidance[2];
} // end of function prediction_from_monitor_geo_fence()

// function to publish final command_geometry_twist through the publisher via this monitor
void publish_final_command_geo_fence()
{
    //ROS_INFO("\n\n------------------------------------Data received----------------------------------------\n\n");

    // make prediction at set frequency
    prediction_from_monitor_geo_fence();
    /*
    ROS_INFO("\n\nGeo fence limits set :\n\n"
    "Fence limit set in direction positive x : %f \n"
    "Fence limit set in direction negative x : %f \n"
    "Fence limit set in direction positive y : %f \n"
    "Fence limit set in direction negative y : %f \n"
    "Fence limit set in direction positive z : %f \n"
    "Fence limit set in direction negative z : %f \n\n"
    "Critical radius (calculating from home location) : %f\n"
    "Critical radius from fence limit : %f\n"
    "Radius of circle of influence \"s\" : %f\n",
    max_possible_pose_in_positive_x, max_possible_pose_in_negative_x,
    max_possible_pose_in_positive_y, max_possible_pose_in_negative_y,
    max_possible_pose_in_positive_z, max_possible_pose_in_negative_z,
    critical_radius_start_from_home, critical_radius_from_fence_limit, radius_of_circle_of_influence_s);
    
    ROS_INFO("\n\nHome location - latitude : %f\n"
    "Home location - longitude : %f\n"
    "Home location - altitude : %f\n\n",
    location_home_lat_x, location_home_long_y, location_home_alt_z);
    
    ROS_INFO("\n\nLocal position received from 'Home' in direction x : %f \n"
    "Local position received from \"Home\" in direction y : %f \n"
    "Local position received from \"Home\" in direction z : %f \n",
    array_local_position_pose_data[0], array_local_position_pose_data[1], array_local_position_pose_data[2]);
    
    ROS_INFO("\n\nDistance between fence limit and vehicle in direction x : %f\n"
    "Distance between fence limit and vehicle in direction y : %f\n"
    "Distance between fence limit and vehicle in direction z : %f\n",
    dist_bet_fence_and_vehicle_x, dist_bet_fence_and_vehicle_y, dist_bet_fence_and_vehicle_z);

    ROS_INFO("\n\nCurrent waypoint set (Index in waypoint list) : %d\n"
    "If updated waypoint was set successfully : %d\n\n",
    command_waypoint_set_current.request.wp_seq, command_waypoint_set_current.response.success);

    ROS_INFO("\n\nCalculated wp in x : %d\n"
    "Calculated wp in y : %d\n\n",
    wp_x, wp_y);
    /*
    ROS_INFO("\n\nDistance between fence and vehicle overall : %f\n"
    "Current angle between fence and vehicle (in radians) : %f\n"
    "Gradient x : %f\n"
    "Gradient y : %f\n"
    "Resulting velocity of vehicle : %f\n"
    "Resulting angle theta : %f\n",
    dist_bet_fence_and_vehicle_overall, angle_bet_fence_and_vehicle,
    gradient_x, gradient_y, resulting_velocity_of_vehicle,
    resulting_angle_theta);
    
    ROS_INFO("\n\nDesired airspeed received via controller in direction x : %f \n""Desired airspeed received via controller in direction y : %f \n"
    "Desired airspeed received via controller in direction z : %f \n", array_velocity_guidance[0], array_velocity_guidance[1],
    array_velocity_guidance[2]);
    
    ROS_INFO("\n\nCurrent mode set request : %s\n"
    "Requested mode actually set : %d\n"
    "Value of constant beta : %f\n",
    command_mavros_set_mode.request.custom_mode.c_str(),
    command_mavros_set_mode.response.mode_sent,
    constant_beta);
    
    ROS_INFO("\n\nFence breach in direction x : %s \n"
    "Fence breach in direction y : %s \n"
    "Fence breach in direction z : %s \n",
    array_monitor_geo_fence_triggered[0].c_str(),
    array_monitor_geo_fence_triggered[1].c_str(),
    array_monitor_geo_fence_triggered[2].c_str());
    
    ROS_INFO("\n\nCorrected airspeed published by monitor in direction x : %f \n"
    "Corrected airspeed published by monitor in direction y : %f \n"
    "Corrected airspeed published by monitor in direction z : %f \n",
    command_geometry_twist.twist.linear.x, command_geometry_twist.twist.linear.y, command_geometry_twist.twist.linear.z);
    ROS_INFO("\n\n\n---------------------------------------------------------------------------------------------\n");
    */
    //srv_set_current_waypoint.call(command_waypoint_set_current);
    pub_corrected_velocity.publish(command_geometry_twist);
    //ROS_INFO("Data publishing to topic \"/mavros/setpoint_velocity/cmd_vel\".");

   // ROS_INFO("\n\n------------------------------------End of data block----------------------------------------\n\n");
} // end of function publish_final_command_geo_fence()
