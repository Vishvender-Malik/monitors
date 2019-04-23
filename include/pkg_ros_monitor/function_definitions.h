/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : function_definitions.h
*/

#ifndef header_function_definitions
#define header_function_definitions

#include "headers.h"
#include "variables.h"

namespace function{
    void get_guidance_controller_velocity(const geometry_msgs::TwistStamped::ConstPtr& data);
    void get_global_position_uav(const sensor_msgs::NavSatFix::ConstPtr& data);
    void get_waypoint_list(const mavros_msgs::WaypointList::ConstPtr& list);
    void convert_lat_long_to_x_y(double x_lat_home, double y_long_home, double x_lat_mission_wp, 
    double long_y_mission_wp);
    void get_waypoint_list_plane(const mavros_msgs::WaypointList::ConstPtr& list);
    void convert_lat_long_to_x_y_plane(double x_lat_home, double y_long_home, double x_lat_mission_wp, double long_y_mission_wp);
    double find_bearing(double wp_x, double wp_y);
    void xy_2latlon(double x_lat_home, double y_long_home, int wp_x, int wp_y, double bearing);
}

#endif