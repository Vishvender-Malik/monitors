/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : function_definitions.cpp
*/

#include "function_definitions.h"
#include "macros.h"

namespace function{
    //<-------------------------------------------Global variables and structures--------------------------------------------------->

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
    wp_y = 0.0, array_global_position_uav[array_global_position_uav_size] = {0.0}, theta_plane, old_wp_x, old_wp_y,
    loiter_wp_x, loiter_wp_y, bearing, x_to_lat, y_to_long;

    int sign_vehicle_pose_x, sign_vehicle_pose_y, sign_vehicle_pose_z, size_waypoint_list, waypoint_current, waypoint_old, waypoint_new;
    bool home_init = true, first_conversion = true, list_trigger = true, corrected_auto = false, start_guided = true, flag = true,
    monitor_triggered = false;

    waypoint array_waypoint_list[array_waypoint_list_size] = {0.0}; // array of struct waypoint

    std::vector<float> vec_wp_lat;
    std::vector<float> vec_wp_long;
    std::vector<float> vec_wp_alt;
    std::vector<mavros_msgs::Waypoint> vec_waypoint_table;

    geometry_msgs::TwistStamped command_geometry_twist; // final command_geometry_twist message to be published
    mavros_msgs::Waypoint message_waypoint; // waypoint table message

    nav_msgs::Odometry command_nav_pose; // final command_nav_pose message to be published
    mavros_msgs::SetMode command_mavros_set_mode; // final command_mavros_set_mode to be published
    mavros_msgs::SetMode command_mavros_set_mode_auto;
    mavros_msgs::WaypointSetCurrent command_waypoint_set_current; // final command to publish updated waypoint 
    mavros_msgs::WaypointPush command_waypoint_push;// wp message to be pushed

    //******************************************************************************************************************************

    // function to receive Desired airspeed (published on to the topic) from the topic itself
    void get_guidance_controller_velocity(const geometry_msgs::TwistStamped::ConstPtr& data)
    {
        array_velocity_guidance[0] = data -> twist.linear.x;
        array_velocity_guidance[1] = data -> twist.linear.y;
        array_velocity_guidance[2] = data -> twist.linear.z;
        ROS_INFO("Data received from topic \"/mavros/local_position/velocity\".");
    }

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

        array_local_position_pose_data[0] = wp_x / 100;
        array_local_position_pose_data[1] = wp_y / 100;
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

    // function to receive mission waypoints
    void get_waypoint_list_plane(const mavros_msgs::WaypointList::ConstPtr& list)
    {
        if(waypoint_current != list -> current_seq){
            waypoint_old = waypoint_current;
            waypoint_new = list -> current_seq;
            waypoint_current = waypoint_new;
        }
        // to store list just one time
        if(list_trigger){
            waypoint_old = waypoint_current;
            size_waypoint_list = list -> waypoints.size();

            for(int i = 0; i < size_waypoint_list; i++){
                // populate array to store wp list
                array_waypoint_list[i].x_lat = list -> waypoints[i].x_lat;
                array_waypoint_list[i].y_long = list -> waypoints[i].y_long;
                array_waypoint_list[i].z_alt = list -> waypoints[i].z_alt;
                
                // populate a waypoint message to be put into table
                message_waypoint.command = 16; // uint16 NAV_WAYPOINT = 16, # Navigate to waypoint
                message_waypoint.param1 = 0.0; // No. of turns by UAV at wp
                message_waypoint.param2 = 0.0;
                message_waypoint.param3 = 0.0;
                message_waypoint.param4 = 0.0;
                message_waypoint.x_lat = list -> waypoints[i].x_lat;
                message_waypoint.y_long = list -> waypoints[i].y_long;
                message_waypoint.z_alt = list -> waypoints[i].z_alt;
                /*
                // populate vector to create editable wp table
                if(i == 0){
                    //vec_waypoint_table.insert(vec_waypoint_table.begin(), message_waypoint);
                }else{
                    vec_waypoint_table.push_back(message_waypoint);
                }*/
                vec_waypoint_table.push_back(message_waypoint);
            }
            list_trigger = false;
        }
        // convert current waypoint's lat long coordinates to x y coordinates
        convert_lat_long_to_x_y(location_home_lat_x, location_home_long_y, array_waypoint_list[waypoint_current].x_lat,
        array_waypoint_list[waypoint_current].y_long);
        // assuming home location to be 0, 0
        ROS_INFO("Data received from topic \"/mavros/mission/waypoints\".");
        ROS_INFO("No. of waypoints received : %d\n"
        "Old waypoint : %d\n"
        "New waypoint : %d\n"
        "Current waypoint : %d\n\n"
        "wp_x : %f\n""wp_y : %f\n",
        size_waypoint_list, waypoint_old, waypoint_new, waypoint_current, wp_x, wp_y);
    }

    // function to calculate variable bearing for use in function xy_2latlon
    double find_bearing(double wp_x, double wp_y)
    {
        bearing = atan2(wp_y, wp_x);
        std::cout<<"bearing : "<<bearing<<"\n\n";
        return bearing;
    }

    // function to convert simple x y coordinates of a waypoint to lat long coordinates
    void xy_2latlon(double x_lat_home, double y_long_home, int wp_x, int wp_y, double bearing)
    {
        x_lat_home = x_lat_home * (m_pi / 180); // to radians
        y_long_home = y_long_home * (m_pi / 180);

        std::cout<<"x_lat_home : "<<x_lat_home<<"\n""y_long_home : "<<y_long_home<<"\n";        
        
        some_parameter_d = sqrt(pow(wp_x, 2) + pow(wp_y, 2));
        x_to_lat = asin(sin(x_lat_home) * cos(some_parameter_d / m_radius_earth) + cos(x_lat_home) * sin(some_parameter_d / m_radius_earth)
                    * cos(find_bearing(wp_x, wp_y)));
        y_to_long = y_long_home + atan2(sin(find_bearing(wp_x, wp_y)) * sin(some_parameter_d / m_radius_earth)
                    *cos(x_lat_home), cos(some_parameter_d / m_radius_earth) - sin(x_lat_home) * sin(x_to_lat));

        x_to_lat = abs(x_to_lat * (180 / m_pi)); // to degrees
        y_to_long = abs(y_to_long * (180 / m_pi));

        std::cout<<"wp_x : "<<wp_x<<"\n""wp_y : "<<wp_y<<"\n";
        std::cout<<"x_to_lat (in degrees) : "<<x_to_lat<<"\n""y_to_long (in degrees)  : "<<y_to_long<<"\n";
        std::cout<<"parameter d : "<<some_parameter_d<<"\n"<<"\n""bearing : "<<function::bearing<<"\n\n";
    }
} // end of namespace function
