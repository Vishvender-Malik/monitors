/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : variables.h
*/

// failsafe for if this header is used more than once in
// the same file (will compile it only once)
//#pragma once
#ifndef header_variables
#define header_variables

#include "headers.h"

extern std::string topic_guidance_velocity;
extern std::string topic_local_position_data;
extern std::string topic_corrected_velocity; 
extern std::string topic_current_mavros_state;
extern std::string mode_flight;
extern std::string topic_waypoint_list;
extern std::string topic_home_lat_and_long;
extern std::string topic_global_position_uav;

// struct declared right here, otherwise it's multiple declaration
struct waypoint{
    float x_lat;
    float y_long;
    float z_alt;
};

// same here as structs
static const int array_velocity_guidance_size = 3, array_local_position_pose_data_size = 3, monitor_geo_fence_triggered_size = 3,
array_waypoint_list_size = 100, radius_of_earth = 6371, array_global_position_uav_size = 3, radius_earth = 6371000,
array_waypoints_plane_size = 200;

extern std::string array_monitor_geo_fence_triggered[monitor_geo_fence_triggered_size]; // for geo fence monitor

extern double array_velocity_guidance[array_velocity_guidance_size], array_local_position_pose_data[array_local_position_pose_data_size],
max_possible_pose_in_positive_x, max_possible_pose_in_negative_x, max_possible_pose_in_positive_y, 
max_possible_pose_in_negative_y, max_possible_pose_in_positive_z, max_possible_pose_in_negative_z,
fence_limit_to_consider_in_x, fence_limit_to_consider_in_y, fence_limit_to_consider_in_z,
dist_bet_fence_and_vehicle_x, dist_bet_fence_and_vehicle_y, dist_bet_fence_and_vehicle_z,
critical_radius_from_fence_limit, radius_of_circle_of_influence_s, dist_bet_fence_and_vehicle_overall,
angle_bet_fence_and_vehicle, gradient_x, gradient_y, constant_beta, resulting_velocity_of_vehicle,
resulting_angle_theta, critical_radius_start_from_home, constant_beta_x,constant_beta_y,
x_lat_home, y_long_home, x_lat_mission_wp, y_long_mission_wp, diff_in_lat, diff_in_long,
some_parameter_a, some_parameter_c, some_parameter_d, some_parameter_y, some_parameter_x, some_parameter_bearing,
waypoint_current_lat, waypoint_current_long, location_home_lat_x, location_home_long_y, location_home_alt_z, wp_x, 
wp_y, array_global_position_uav[array_global_position_uav_size], theta_plane, old_wp_x, old_wp_y,
loiter_wp_x, loiter_wp_y, bearing, x_to_lat, y_to_long;

extern int sign_vehicle_pose_x, sign_vehicle_pose_y, sign_vehicle_pose_z, size_waypoint_list, waypoint_current, waypoint_old, waypoint_new;

extern std::vector<float> vec_wp_lat;
extern std::vector<float> vec_wp_long;
extern std::vector<float> vec_wp_alt;
extern std::vector<mavros_msgs::Waypoint> vec_waypoint_table;

extern bool home_init, first_conversion, list_trigger, corrected_auto, start_guided, flag,
monitor_triggered;

extern waypoint array_waypoint_list[array_waypoint_list_size]; // array of struct waypoint
extern mavros_msgs::Waypoint array_waypoints_plane[array_waypoints_plane_size];

extern geometry_msgs::TwistStamped command_geometry_twist; // final command_geometry_twist message to be published
extern mavros_msgs::Waypoint message_waypoint; // waypoint table message

extern nav_msgs::Odometry command_nav_pose; // final command_nav_pose message to be published
extern mavros_msgs::SetMode command_mavros_set_mode; // final command_mavros_set_mode to be published
extern mavros_msgs::SetMode command_mavros_set_mode_auto;
extern mavros_msgs::WaypointSetCurrent command_waypoint_set_current; // final command to publish updated waypoint 
extern mavros_msgs::WaypointPush command_waypoint_push;// wp message to be pushed

// publishers, subscribers and services for monitor_geo_fence_plane
extern ros::Subscriber sub_guidance_velocity;
extern ros::Publisher pub_corrected_velocity;
extern ros::Subscriber sub_local_position_data;
extern ros::Subscriber sub_current_mavros_state;
extern ros::ServiceClient srv_mavros_state;
extern ros::Subscriber sub_waypoint_list;
extern ros::Subscriber sub_waypoint_list_plane;
extern ros::Subscriber sub_home_lat_and_long;
extern ros::ServiceClient srv_set_current_waypoint;
extern ros::Subscriber sub_global_position_uav;
extern ros::ServiceClient srv_wp_push;

#endif