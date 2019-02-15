/* Author : Vishvender Malik
Email : vishvenderm@iiitd.ac.in
File : node_monitor_wind.cpp
*/

#include "monitor_wind.h"

//-------------------------------------------------------------------------------------------------------------------------------

//<-------------------------------------------Global variables and structures--------------------------------------------------->

std::string topic_guidance_velocity = "";
std::string topic_desired_airspeed = "";
std::string topic_receive_altitude = "";
std::string topic_wind_estimation = "";
std::string topic_blobDataFlags = "";
std::string topic_corrected_airspeed = "";

const int array_velocity_guidance_size = 3, array_wind_velocity_size = 3, array_guidance_velocity_size = 3,
array_total_irl_vel_size = 3, size_vel_difference = 3, size_covariance = 36, array_new_airspeed_size = 3;

double array_velocity_guidance[array_velocity_guidance_size] = {0}, array_windspeed[array_wind_velocity_size] = {0},
array_groundspeed_guidance[array_guidance_velocity_size] = {0}, array_actual_groundspeed[array_total_irl_vel_size] = {0},
array_vel_difference[size_vel_difference] = {0}, array_new_airspeed[array_new_airspeed_size] = {0};

int flag_target_detected, flag_target_tracking;

float altitude = 0.0, array_covariance[size_covariance], sign_groundspeed_guidance_vel_x, sign_groundspeed_guidance_vel_y, 
sign_groundspeed_guidance_vel_z, sign_actual_groundspeed_vel_x, sign_actual_groundspeed_vel_y, sign_actual_groundspeed_vel_z, 
max_possible_vel_in_positive_x, max_possible_vel_in_positive_y, max_possible_vel_in_z, max_possible_vel_in_negative_x, 
max_possible_vel_in_negative_y, max_possible_vel_in_positive_z, max_possible_vel_in_negative_z, 
sign_new_airspeed_vel_x, sign_new_airspeed_vel_y,
sign_new_airspeed_vel_z, sign_local_position_y, target_center_x, target_center_y;

pkg_ros_monitor::monitor_Config config; 
uint32_t level;

geometry_msgs::TwistStamped command_geometry_twist; // final command_geometry_twist message to be published

// publishers and subscribers for monitor_wind
ros::Publisher pub_landing_target_info;
ros::Publisher pub_corrected_airspeed;
ros::Subscriber sub_groundspeed_guidance;
ros::Subscriber sub_guidance_velocity;
ros::Subscriber sub_receive_altitude;
ros::Subscriber sub_get_windspeed;
ros::Subscriber sub_vision_landing_target_info;

//<----------------------------------------------------------------------------------------------------------------------------->

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

//<----------------------------------------------------------------------------------------------------------------------------->

//<------------------------------------------------Initialize monitor node------------------------------------------------------>

int main(int argc, char **argv)
{
    ROS_INFO("\n\n----------------Welcome-----------------\n\n");

    // initialize ros node with a node name
    ros::init(argc, argv, "node_monitor_wind");
    
    // create monitor object
    monitor_wind obj_monitor_wind;
    // implement class functions
    obj_monitor_wind.init_parameter_server();
    //obj_monitor_wind.set_monitor_topics(config, level); // main issue is here, not using same config as before
    obj_monitor_wind.initialize_pub_and_sub();
    obj_monitor_wind.monitor_start();
    
    return 0;

} // end of initialization

//<------------------------------------------Function definitions---------------------------------------------------------------->

monitor_wind::monitor_wind() : monitor_base(){
    ROS_INFO("monitor_wind object initialized\n");
}

void monitor_wind::init_parameter_server(){
    ROS_INFO("init_parameter_server function called.\n");
    // define our parameter server, and pass it our configuration file information
    // as long as the server lives (in this case until the end of our node, the monitor node listens to reconfigure requests
    //dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config> parameter_server;

    // define a variable to represent our callback object and provide it info about our callback function
    //dynamic_reconfigure::Server<pkg_ros_monitor::monitor_Config>::CallbackType callback_variable;
    //this -> set_monitor_topics(config, level);
    callback_variable = boost::bind(&monitor_base::set_monitor_topics, this, _1, _2);

    // pass our callback object to parameter server
    // now when the server gets a reconfiguration request it will call our callback function
    // call the callback function at least just one time before initilizing subscribers
    parameter_server.setCallback(callback_variable); 
    //ROS_INFO("Spinning parameter server.\n");
    //ros::spinOnce();
    ROS_INFO("init_parameter_server function ended.\n");
    //this -> initialize_pub_and_sub();
}

void monitor_wind::set_monitor_topics(pkg_ros_monitor::monitor_Config &config, uint32_t level){
    
    ROS_INFO("set_monitor_topics function reached\n");
    
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

    ROS_INFO("set_monitor_topics function ended\n");
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

void monitor_wind::monitor_start(){
   
    ROS_INFO("start of monitor_start function reached");
    // get single wind monitor instance
    //monitor_wind::getInstance().initialize_pub_and_sub();
    //ROS_INFO("monitor_wind::getInstance function executed");
    //monitor_wind::initialize_pub_and_sub();

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
    ROS_INFO("end of monitor_start function reached");
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
    
    ROS_INFO("\n\nDesired airspeed received via topic in direction x : %f \n""Desired airspeed received via topic in direction y : %f \n"
    "Desired airspeed received via topic in direction z : %f \n", array_velocity_guidance[0], array_velocity_guidance[1],
    array_velocity_guidance[2]);

    ROS_INFO("\n\nWind speed received via topic in direction x : %f \n""Wind speed received via topic in direction y : %f \n"
    "Wind speed received via topic in direction z : %f \n", array_windspeed[0], array_windspeed[1],
    array_windspeed[2]);
    
    ROS_INFO("\n\nAltitude Info received from vision controller : %f\n", altitude);

    ROS_INFO("\n\nTarget distance received from vision controller in direction x : %f \n"
    "Target distance received from vision controller in direction y : %f \n", target_center_x, target_center_y);

    ROS_INFO("\n\nFlag value for target being detected : %d \n"
    "Flag value for target being tracked : %d \n", flag_target_detected, flag_target_tracking);

    ROS_INFO("\n\nGroundspeed received for comparison from guidance controller in direction x : %f \n"
    "Groundspeed received for comparison from guidance controller in direction y : %f \n"
    "Groundspeed received for comparison from guidance controller in direction z : %f\n",
    array_groundspeed_guidance[0], array_groundspeed_guidance[1], array_groundspeed_guidance[2]);

    // make prediction at set frequency
    prediction_from_monitor_wind();
    
    ROS_INFO("\n\nCorrected airspeed published by monitor in direction x : %f \n"
    "Corrected airspeed published by monitor in direction y : %f \n"
    "Corrected airspeed published by monitor in direction z : %f\n\n",
    command_geometry_twist.twist.linear.x, command_geometry_twist.twist.linear.y, command_geometry_twist.twist.linear.z);
    ROS_INFO("\n\n\n---------------------------------------------------------------------------------------------\n");

    // finally, publish to the topic
    pub_corrected_airspeed.publish(command_geometry_twist);
    ROS_INFO("Data publishing to topic \"/mavros/setpoint_velocity/cmd_vel_new\".");
    ROS_INFO("\n\n------------------------------------End of data block----------------------------------------\n\n");
} // end of function publish_final_command_wind()