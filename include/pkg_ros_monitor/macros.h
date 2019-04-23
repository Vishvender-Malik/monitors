/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : macros.h
*/
/******************************************
 * Has macros for constant values and publisher and subscriber declarations
*/

#ifndef header_macros
#define header_macros

#include "function_definitions.h"

#define m_pi 3.14159265
#define m_radius_of_earth 6371
#define m_radius_earth 6371000
#define m_queue_sub 1000
#define m_queue_pub 1000
#define m_array_velocity_guidance_size 3
#define m_array_local_position_pose_data_size 3
#define m_monitor_geo_fence_triggered_size 3
#define m_array_waypoint_list_size 100
#define m_array_global_position_uav_size 3

//*****************COMMON SUBSCRIBERS*******************
#define m_sub_guidance_velocity nodeHandle.subscribe(topic_guidance_velocity, m_queue_sub, function::get_guidance_controller_velocity)
#define m_sub_global_position_uav nodeHandle.subscribe(topic_global_position_uav, m_queue_sub, function::get_global_position_uav)

//*****************QUADROTOR SUBSCRIBERS****************************
#define m_sub_waypoint_list nodeHandle.subscribe(topic_waypoint_list, m_queue_sub, function::get_waypoint_list)

//******************FIXED WING PLANE SUBSCRIBERS***************************
#define m_sub_waypoint_list_plane nodeHandle.subscribe(topic_waypoint_list, 1000, function::get_waypoint_list_plane)

//****************SERVICES*****************************
#define m_srv_mavros_state nodeHandle.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode")
#define m_srv_current_waypoint nodeHandle.serviceClient<mavros_msgs::WaypointSetCurrent>("/mavros/mission/set_current")
#define m_srv_waypoint_push nodeHandle.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push")

//****************PUBLISHERS*****************************
#define m_pub_corrected_velocity nodeHandle.advertise<geometry_msgs::TwistStamped>(topic_corrected_velocity, m_queue_pub)

#endif