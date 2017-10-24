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
#include "std_msgs/Int32MultiArray.h"

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
            float angular_velocity = 0.2;
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
    RoverPose rover;
    if (rover_name == "achilles") {
        rover.name = ACHILLES;
    } else if (rover_name == "aeneas") {
        rover.name = AENEAS;
    } else if (rover_name == "ajax") {
        rover.name = AJAX;
    }
    rover_pose.data.clear();
    rover_pose.data.push_back(rover.name);
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
//    Create Hashmap
//    rover_hash = new std::map<int, RoverPose>;
//    Create vector to pass to RoverPose
    std::cout << "Start headingHandler" << std::endl;
    std::vector<double> message_data;
    std::cout << "Pushing on message_data vector" << std::endl;
    message_data.push_back(message->data[1]);
    message_data.push_back(message->data[2]);
    message_data.push_back(message->data[3]);
    std::cout << "Pushed" << std::endl;
//    Create RoverPose
    std::cout << "Create RoverPose" << std::endl;
    RoverPose rover_data((int) message->data[0], message_data);
//    Populate Hash
    std::cout << "Create rover_hash" << std::endl;
    rover_hash[(int) message->data[0]] = rover_data;
    std::cout << "*** Heading Handler: Name " << rover_hash[(int) message->data[0]].name << std::endl;
    std::cout << "*** Heading Handler: X " << rover_hash[(int) message->data[0]].x << std::endl;
    std::cout << "*** Heading Handler: Y " << rover_hash[(int) message->data[0]].y << std::endl;
    std::cout << "*** Heading Handler: Theta " << rover_hash[(int) message->data[0]].theta << std::endl;

//    Call globalHeading
    std_msgs::String gH = globalHeading();
    std::cout << "headingHandler End" << std::endl;
    globalAverageHeading.publish(gH);

//    Call neighbors
    neighbors();

//    Call localHeading
    std_msgs::String lH = localHeading();
    localAverageHeading.publish(lH);
}
std_msgs::String globalHeading (){
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double gAH;
//    Iterate through hash and populate thetaG with theta values
    std::cout << "Before iteration" << std::endl;
    for (std::map<int, RoverPose>::iterator it = rover_hash.begin(); it != rover_hash.end(); ++it){
        std::cout << "---> Rover Name: " << it->second.name << std::endl;
        std::cout << "---> Rover Theta: " << it->second.theta << std::endl;
        thetaG.at(0) += cos(it->second.theta);
        thetaG.at(1) += sin(it->second.theta);
    }
    std::cout << "After iteration" << std::endl;
    std::cout << "Next iteration" << std::endl;
    thetaG.at(0) /= rover_hash.size();
    thetaG.at(1) /= rover_hash.size();

    std::cout << "Done iterating" << std::endl;
    gAH = std::atan2(thetaG[1], thetaG[0]);\
    std::cout << "Calculated gAH " << gAH << std::endl;
//    Format string
    snprintf(buf, 256, "Global Heading: %lf", gAH);
    content.data = string(buf);
    memset(buf, 0, 255); // Clear char array
    std::cout << "globalHeading End" << std::endl;
    return content;
}

void neighbors () {
    double d01 = sqrt(pow(rover_hash[0].x - rover_hash[1].x, 2) + pow(rover_hash[0].y - rover_hash[1].y, 2));
    double d02 = sqrt(pow(rover_hash[0].x - rover_hash[2].x, 2) + pow(rover_hash[0].y - rover_hash[2].y, 2));
    double d12 = sqrt(pow(rover_hash[1].x - rover_hash[2].x, 2) + pow(rover_hash[1].y - rover_hash[2].y, 2));
//    Empty neighbors vector
    for (std::map<int, RoverPose>::iterator it = rover_hash.begin(); it != rover_hash.end(); ++it){
        rover_hash[it->first].neighbors.clear();
    }
//    All rovers are neighbors
    if (d01 < NEIGH_DIST && d02 < NEIGH_DIST && d12 < NEIGH_DIST){
        rover_hash[0].neighbors.push_back(1);
        rover_hash[0].neighbors.push_back(2);
        rover_hash[1].neighbors.push_back(0);
        rover_hash[1].neighbors.push_back(2);
        rover_hash[2].neighbors.push_back(0);
        rover_hash[2].neighbors.push_back(1);
    }
//    Only two are neighbors
    else if ((d01 < NEIGH_DIST && d02 < NEIGH_DIST) && d12 >= NEIGH_DIST){
        rover_hash[0].neighbors.push_back(1);
        rover_hash[0].neighbors.push_back(2);
        rover_hash[1].neighbors.push_back(0);
        rover_hash[2].neighbors.push_back(0);
    }
    else if ((d01 < NEIGH_DIST && d12 < NEIGH_DIST) && d02 >= NEIGH_DIST){
        rover_hash[1].neighbors.push_back(0);
        rover_hash[1].neighbors.push_back(2);
        rover_hash[0].neighbors.push_back(1);
        rover_hash[2].neighbors.push_back(1);
    }
    else if ((d12 < NEIGH_DIST && d02 < NEIGH_DIST) && d01 >= NEIGH_DIST){
        rover_hash[2].neighbors.push_back(0);
        rover_hash[2].neighbors.push_back(1);
        rover_hash[0].neighbors.push_back(2);
        rover_hash[1].neighbors.push_back(2);
    }
}


std_msgs::String localHeading () {
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double lAH;
    ROVER_POSE rover;
    std::vector<double> rNeighb;

//    Decide which rover we are assigning neighbors to
    if (rover_name == "achilles") {
        rover.name = ACHILLES;
    } else if (rover_name == "aeneas") {
        rover.name = AENEAS;
    } else if (rover_name == "ajax") {
        rover.name = AJAX;
    }
//    Set flagged values
    thetaG.at(0) = 10; // values outside of range
    thetaG.at(0) = 10;
//    Iterate through respective neighbors vector
    for (std::vector<int>::iterator it = rover_hash[rover.name].neighbors.begin(); it != rover_hash[rover.name].neighbors.end(); ++it){
        rNeighb.push_back(*it);
    }

    if (!rNeighb.empty()){
//        Iterator through neighboring rovers pose
        for (std::vector<double>::iterator it = rNeighb.begin(); it != rNeighb.end(); ++it){
            thetaG.at(0) += cos(rover_hash[*it].theta);
            thetaG.at(1) += sin(rover_hash[*it].theta);
        }
        thetaG.at(0) /= thetaG.size();
        thetaG.at(1) /= thetaG.size();
        lAH = std::atan2(thetaG[1], thetaG[0]);
    }
//    No neighbors
    else {
        lAH = 0;
    }

//    Format string
    snprintf(buf, 256, "Local Heading: %lf", lAH);
    content.data = string(buf);
    memset(buf, 0, 255); // Clear char array

    return content;
}
