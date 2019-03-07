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
    obj_monitor_geo_fence.initialize_pub_and_sub();
    obj_monitor_geo_fence.monitor_start();
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

    // final publisher to application // topic should just be cmd_vel
    pub_corrected_velocity = nodeHandle.advertise<geometry_msgs::TwistStamped>(topic_corrected_velocity, 1000);

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

    //run loop ideally at (10) Hz (always in decimal and faster than what is published through guidance controller)
    ros::Rate loop_rate(1.00);
    
    // Use current time as seed for random generator 
    //srand(time(0)); 

    while (ros::ok())
    {
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

// function to calculate required predictions and populate "command_nav_pose" message
void prediction_from_monitor_geo_fence()
{
    /*
    APPROACH : 
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

    // action to be taken
    
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
    } // end of outer if 
   
} // end of function prediction_from_monitor_geo_fence()

// function to publish final command_geometry_twist through the publisher via this monitor
void publish_final_command_geo_fence()
{
    // make prediction at set frequency
    prediction_from_monitor_geo_fence();
    
    pub_corrected_velocity.publish(command_geometry_twist);
} // end of function publish_final_command_geo_fence()
