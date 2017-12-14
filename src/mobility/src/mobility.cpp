#include <ros/ros.h>

// ROS libraries
#include <angles/angles.h>
#include <random_numbers/random_numbers.h>
#include <tf/transform_datatypes.h>

// ROS messages
#include <std_msgs/Int16.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Range.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include "Pose.h"
#include "TargetState.h"

// Custom messages
#include <shared_messages/TagsImage.h>

// To handle shutdown signals so the node quits properly in response to "rosnode kill"
#include <signal.h>


#include "mobility.h"
using namespace std;

#include <map>
#include <math.h>
#include <list>

// Random number generator
random_numbers::RandomNumberGenerator *rng;

string rover_name;
char host[128];
bool is_published_name = false;

int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

pose current_location;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

int main(int argc, char **argv)
{
    gethostname(host, sizeof(host));
    string hostName(host);

    rng = new random_numbers::RandomNumberGenerator(); // instantiate random number generator

    if (argc >= 2)
    {
        rover_name = argv[1];
        cout << "Welcome to the world of tomorrow " << rover_name << "!  Mobility module started." << endl;
    } else
    {
        rover_name = hostName;
        cout << "No Name Selected. Default is: " << rover_name << endl;
    }
    // NoSignalHandler so we can catch SIGINT ourselves and shutdown the node
    ros::init(argc, argv, (rover_name + "_MOBILITY"), ros::init_options::NoSigintHandler);
    ros::NodeHandle mNH;

    signal(SIGINT, sigintEventHandler); // Register the SIGINT event handler so the node can shutdown properly

    joySubscriber = mNH.subscribe((rover_name + "/joystick"), 10, joyCmdHandler);
    modeSubscriber = mNH.subscribe((rover_name + "/mode"), 1, modeHandler);
    targetSubscriber = mNH.subscribe((rover_name + "/targets"), 10, targetHandler);
    obstacleSubscriber = mNH.subscribe((rover_name + "/obstacle"), 10, obstacleHandler);
    odometrySubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, odometryHandler);
    messageSubscriber = mNH.subscribe(("messages"), 10, messageHandler);
    /*
     * Added subsciber instantiation
     */
    headingSubscriber = mNH.subscribe(("/currPose"), 10, headingHandler);

    status_publisher = mNH.advertise<std_msgs::String>((rover_name + "/status"), 1, true);
    velocityPublish = mNH.advertise<geometry_msgs::Twist>((rover_name + "/velocity"), 10);
    stateMachinePublish = mNH.advertise<std_msgs::String>((rover_name + "/state_machine"), 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10, true);
    target_collected_publisher = mNH.advertise<std_msgs::Int16>(("targetsCollected"), 1, true);
    angular_publisher = mNH.advertise<std_msgs::String>((rover_name + "/angular"),1,true);
    publish_status_timer = mNH.createTimer(ros::Duration(status_publish_interval), publishStatusTimerEventHandler);
    killSwitchTimer = mNH.createTimer(ros::Duration(kill_switch_timeout), killSwitchTimerEventHandler);
    stateMachineTimer = mNH.createTimer(ros::Duration(mobility_loop_time_step), mobilityStateMachine);
    debug_publisher = mNH.advertise<std_msgs::String>("/debug", 1, true);
    messagePublish = mNH.advertise<std_msgs::String>(("messages"), 10 , true);
    /*
     * Added advertisers instatiation
     */
    currentPose = mNH.advertise<std_msgs::Float64MultiArray>(("/currPose"), 10, true);
    globalAverageHeading = mNH.advertise<std_msgs::String>(("/global_heading"), 10, true);
    localAverageHeading = mNH.advertise<std_msgs::String>((rover_name + "/local_heading"), 10, true);
    
    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    int rover;
    if (rover_name == "achilles") {
        rover = ACHILLES;
    } else if (rover_name == "aeneas") {
        rover = AENEAS;
    } else if (rover_name == "ajax") {
        rover = AJAX;
    } else if (rover_name == "diomedes") {
        rover = DIOMEDES;
    } else if (rover_name == "hector") {
        rover = HECTOR;
    } else if (rover_name == "paris") {
        rover = PARIS;
    }

    std_msgs::String state_machine_msg;

    if ((simulation_mode == 2 || simulation_mode == 3)) // Robot is in automode
    {
        if (transitions_to_auto == 0)
        {
            // This is the first time we have clicked the Autonomous Button. Log the time and increment the counter.
            transitions_to_auto++;
            time_stamp_transition_to_auto = ros::Time::now().toSec();
        }
        switch (state_machine_state)
        {
        case STATE_MACHINE_TRANSLATE:
        {
            state_machine_msg.data = "TRANSLATING";//, " + converter.str();
            double angular_velocity = 0.3 * (rover_hash[rover].avg_local_pose - current_location.theta) +
                    0.3 * (rover_hash[rover].avg_local_theta - current_location.theta) +
                    0.1 * (rover_hash[rover].separation - current_location.theta);
            std::cout << "Angular velecity: " << angular_velocity << std::endl;
            float linear_velocity = 0.1;
            setVelocity(linear_velocity, angular_velocity);
            break;
        }
        default:
        {
            state_machine_msg.data = "DEFAULT CASE: SOMETHING WRONG!!!!";
            break;
        }
        }

    }
    else
    { // mode is NOT auto

        // publish current state for the operator to seerotational_controller
        std::stringstream converter;
        converter <<"CURRENT MODE: " << simulation_mode;

        state_machine_msg.data = "WAITING, " + converter.str();
    }

    /*
     * Create current pose publisher for each rover
     */
    std_msgs::Float64MultiArray rover_pose;
    rover_pose.data.clear();
    rover_pose.data.push_back(rover);
    rover_pose.data.push_back(current_location.x);
    rover_pose.data.push_back(current_location.y);
    rover_pose.data.push_back(current_location.theta);
    currentPose.publish(rover_pose);


    stateMachinePublish.publish(state_machine_msg);
}

void setVelocity(double linearVel, double angularVel)
{
    geometry_msgs::Twist velocity;
    // Stopping and starting the timer causes it to start counting from 0 again.
    // As long as this is called before the kill switch timer reaches kill_switch_timeout seconds
    // the rover_refs's kill switch wont be called.
    killSwitchTimer.stop();
    killSwitchTimer.start();

    velocity.linear.x = linearVel * 1.5;
    velocity.angular.z = angularVel * 8; //scaling factor for sim; removed by aBridge node
    velocityPublish.publish(velocity);
}

/***********************
 * ROS CALLBACK HANDLERS
 ************************/
void targetHandler(const shared_messages::TagsImage::ConstPtr &message) {
    // Only used if we want to take action after seeing an April Tag.
}

void modeHandler(const std_msgs::UInt8::ConstPtr &message)
{
    simulation_mode = message->data;
    setVelocity(0.0, 0.0);
}

void obstacleHandler(const std_msgs::UInt8::ConstPtr &message)
{
    if ( message->data > 0 )
    {
        if (message->data == 1)
        {
            // obstacle on right side
        }
        else
        {
            //obstacle in front or on left side
        }
    }
}

void odometryHandler(const nav_msgs::Odometry::ConstPtr &message)
{
    //Get (x,y) location directly from pose
    current_location.x = message->pose.pose.position.x;
    current_location.y = message->pose.pose.position.y;

    //Get theta rotation by converting quaternion orientation to pitch/roll/yaw
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_location.theta = yaw;
}

void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message)
{
    if (simulation_mode == 0 || simulation_mode == 1)
    {
        setVelocity(message->linear.x, message->angular.z);
    }
}

void publishStatusTimerEventHandler(const ros::TimerEvent &)
{
    if (!is_published_name)
    {
        std_msgs::String name_msg;
        name_msg.data = "I ";
        name_msg.data = name_msg.data + rover_name;
        messagePublish.publish(name_msg);
        is_published_name = true;
    }

    std_msgs::String msg;
    msg.data = "online";
    status_publisher.publish(msg);
}

// Safety precaution. No movement commands - might have lost contact with ROS. Stop the rover_refs.
// Also might no longer be receiving manual movement commands so stop the rover_refs.
void killSwitchTimerEventHandler(const ros::TimerEvent &t)
{
    // No movement commands for killSwitchTime seconds so stop the rover_refs
    setVelocity(0.0, 0.0);
    double current_time = ros::Time::now().toSec();
    ROS_INFO("In mobility.cpp:: killSwitchTimerEventHander(): Movement input timeout. Stopping the rover_refs at %6.4f.",
             current_time);
}

void sigintEventHandler(int sig)
{
    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

void messageHandler(const std_msgs::String::ConstPtr& message)
{
}
void headingHandler(const std_msgs::Float64MultiArray::ConstPtr &message) {
    int current_rover = (int) message->data[0];
    geometry_msgs::Pose2D msg_pose;
    msg_pose.x = message->data[1];
    msg_pose.y = message->data[2];
    msg_pose.theta = message->data[3];

//    Create RoverPose
    RoverPose rover_data(msg_pose);
//    Populate Hash
    rover_hash[current_rover] = rover_data;

//    Call globalHeading
    std_msgs::String gH = globalHeading();
    globalAverageHeading.publish(gH);
    char *end;
    rover_hash[current_rover].avg_global_theta = std::strtod(gH.data.c_str(), &end); // Convert to double
    end = NULL; // clear pointer

//    Call neighbors
    neighbors(current_rover);

//    Call localHeading
    std_msgs::String lH = localHeading((int) message->data[0]);
    localAverageHeading.publish(lH);
    rover_hash[current_rover].avg_local_theta = std::strtod(lH.data.c_str(), &end);
    end = NULL; // clear pointer

//    Call localPose
    std_msgs::String lP = localPose((int)  message->data[0]);
    rover_hash[current_rover].avg_local_pose = std::strtod(lP.data.c_str(), &end);
    end = NULL; // clear pointers

//    Call separation
    std_msgs::String sep = separation((int) message->data[0]);
    rover_hash[current_rover].separation = std::strtod(sep.data.c_str(), &end);
    end = NULL; // clear pointers

}


std_msgs::String globalHeading (){
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double gAH;
//    Iterate through hash and populate thetaG with theta values
    for (std::map<int, RoverPose>::iterator it = rover_hash.begin(); it != rover_hash.end(); ++it){
        thetaG.at(0) += cos(it->second.rover_pose.theta);
        thetaG.at(1) += sin(it->second.rover_pose.theta);
    }
    thetaG.at(0) /= rover_hash.size();
    thetaG.at(1) /= rover_hash.size();
    gAH = std::atan2(thetaG[1], thetaG[0]);

//    Format string
    snprintf(buf, 256, "%lf", gAH);
    content.data = string(buf);
    memset(buf, 0, 255); // Clear char array

    return content;
}

void neighbors (int name) {
//    Empty neighbors vector
    for (std::map<int, RoverPose>::iterator it = rover_hash.begin(); it != rover_hash.end(); ++it){
        rover_hash[it->first].neighbors.clear();
    }
//    Hold constant the current rover
    RoverPose curr_rover = rover_hash[name];
//    Iterate through remaining rovers
    for (std::map<int, RoverPose>::iterator it = rover_hash.begin(); it != rover_hash.end(); ++it){
        if ((it->first != name) && (hypot(curr_rover.rover_pose.x - it->second.rover_pose.x, curr_rover.rover_pose.y - it->second.rover_pose.y) < NEIGH_DIST)){
            rover_hash[name].neighbors.push_back(it->first); // save the neighbors to the constant rover
            rover_hash[it->first].neighbors.push_back(name); // save the neighbors to the other rover
        }
    }
}


std_msgs::String localHeading (int name) {
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double lAH;
    std::vector<double> rNeighb;

//    Iterate through respective neighbors vector
    for (std::vector<int>::iterator it = rover_hash[name].neighbors.begin(); it != rover_hash[name].neighbors.end(); ++it){
        rNeighb.push_back(*it);
    }

//    Iterator through neighboring rovers pose
    for (std::vector<double>::iterator it = rNeighb.begin(); it != rNeighb.end(); ++it){
        thetaG.at(0) += cos(rover_hash[*it].rover_pose.theta);
        thetaG.at(1) += sin(rover_hash[*it].rover_pose.theta);
    }
    thetaG.at(0) /= thetaG.size();
    thetaG.at(1) /= thetaG.size();
    lAH = std::atan2(thetaG[1], thetaG[0]);

//    Format string
    snprintf(buf, 256, "%lf", lAH);
    content.data = string(buf);
    memset(buf, 0, 255); // Clear char array

    return content;
}

std_msgs::String localPose (int name) {
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double lAP;
    std::vector<double> rNeighb;

//    Iterate through respective neighbors vector
    for (std::vector<int>::iterator it = rover_hash[name].neighbors.begin(); it != rover_hash[name].neighbors.end(); ++it) {
        rNeighb.push_back(*it);
    }

//    Iterate through neighboring rovers pose
    for (std::vector<double>::iterator it = rNeighb.begin(); it != rNeighb.end(); ++it) {
        thetaG.at(0) += rover_hash[*it].rover_pose.x;
        thetaG.at(1) += rover_hash[*it].rover_pose.y;
    }
    thetaG.at(0) /= thetaG.size();
    thetaG.at(1) /= thetaG.size();
    lAP = std::atan2(thetaG[1], thetaG[0]);

//    Format String
    snprintf(buf, 256, "%lf", lAP);
    content.data = string(buf);
    memset(buf, 0, 255); // clear char array

    return content;
}

std_msgs::String separation (int name)  {
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double sep;
    std::vector<double> rNeighb;

//    Iterate through respective neighbors vector
    for (std::vector<int>::iterator it = rover_hash[name].neighbors.begin(); it != rover_hash[name].neighbors.end(); ++it) {
        rNeighb.push_back(*it);
    }

//    Iterate through neighboring rovers' pose and steer away
    for (std::vector<double>::iterator it = rNeighb.begin(); it != rNeighb.end(); ++it) {
        thetaG.at(0) += rover_hash[*it].rover_pose.x - current_location.x;
        thetaG.at(1) += rover_hash[*it].rover_pose.y - current_location.y;
    }
    thetaG.at(0) /= -thetaG.size();
    thetaG.at(1) /= -thetaG.size();
    sep = std::atan2(thetaG[1], thetaG[0]);

//    Format String
    snprintf(buf, 256, "%lf", sep);
    content.data = string(buf);
    memset(buf, 0, 255); // clear char array

    return content;
}

