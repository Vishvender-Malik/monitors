/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : variables.cpp
*/

#include "variables.h"

//namespace variables{

    std::string topic_guidance_velocity = "";
    std::string topic_local_position_data = "";
    std::string topic_corrected_velocity = ""; 
    std::string topic_current_mavros_state = "";
    std::string mode_flight = "";
    std::string topic_waypoint_list = "";
    std::string topic_home_lat_and_long = "";
    std::string topic_global_position_uav = "";
    /*
    struct waypoint{
        float x_lat;
        float y_long;
        float z_alt;
    };
    */
    //const int array_velocity_guidance_size = 3, array_local_position_pose_data_size = 3, monitor_geo_fence_triggered_size = 3,
    //array_waypoint_list_size = 100, radius_of_earth = 6371, array_global_position_uav_size = 3, radius_earth = 6371000;

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
    wp_y = 0.0, array_global_position_uav[array_global_position_uav_size] = {0.0}, theta_plane, old_wp_x, old_wp_y,
    loiter_wp_x, loiter_wp_y, bearing, x_to_lat, y_to_long;

    int sign_vehicle_pose_x, sign_vehicle_pose_y, sign_vehicle_pose_z, size_waypoint_list, waypoint_current, waypoint_old, waypoint_new;

    std::vector<float> vec_wp_lat;
    std::vector<float> vec_wp_long;
    std::vector<float> vec_wp_alt;
    std::vector<mavros_msgs::Waypoint> vec_waypoint_table;

    bool home_init = true, first_conversion = true, list_trigger = true, corrected_auto = false, start_guided = true, flag = true,
    monitor_triggered = false;

    waypoint array_waypoint_list[array_waypoint_list_size] = {0.0}; // array of struct waypoint
    mavros_msgs::Waypoint array_waypoints_plane[array_waypoints_plane_size];

    geometry_msgs::TwistStamped command_geometry_twist; // final command_geometry_twist message to be published
    mavros_msgs::Waypoint message_waypoint; // waypoint table message

    nav_msgs::Odometry command_nav_pose; // final command_nav_pose message to be published
    mavros_msgs::SetMode command_mavros_set_mode; // final command_mavros_set_mode to be published
    mavros_msgs::SetMode command_mavros_set_mode_auto;
    mavros_msgs::WaypointSetCurrent command_waypoint_set_current; // final command to publish updated waypoint 
    mavros_msgs::WaypointPush command_waypoint_push;// wp message to be pushed

    // publishers, subscribers and services for monitor_geo_fence_plane
    ros::Subscriber sub_guidance_velocity;
    ros::Publisher pub_corrected_velocity;
    ros::Subscriber sub_local_position_data;
    ros::Subscriber sub_current_mavros_state;
    ros::ServiceClient srv_mavros_state;
    ros::Subscriber sub_waypoint_list;
    ros::Subscriber sub_waypoint_list_plane;
    ros::Subscriber sub_home_lat_and_long;
    ros::ServiceClient srv_set_current_waypoint;
    ros::Subscriber sub_global_position_uav;
    ros::ServiceClient srv_wp_push;
 
//}