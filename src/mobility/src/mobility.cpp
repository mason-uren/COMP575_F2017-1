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
#include <math.h>
#include "mobility.h"
#include "Swarm.h"

using namespace std;

// Random number generator
random_numbers::RandomNumberGenerator *rng;

/*
 * ROVER_REFS
 */
//ROVER_REFS rover_refs;
//std::vector<ROVER_REFS> swarm(NUM_ROVERS)
Swarm swarm(NUM_ROVERS);
AGENT_REFS agent;

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
    headingSubscriber = mNH.subscribe((rover_name + "/odom/ekf"), 10, headingHandler);

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
    currentPose = mNH.advertise<std_msgs::String>((rover_name + "/heading"), 10, true);
    globalAverageHeading = mNH.advertise<std_msgs::String>((rover_name + "/global_heading"), 10, true);
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
void headingHandler(const nav_msgs::Odometry::ConstPtr &message) {
    if (rover_name == "achilles") {
        agent.name = ACHILLES;
//        rovers.agent_refs.name = ACHILLES;
    } else if (rover_name == "aeneas") {
        agent.name = AENEAS;
//        rovers.agent_refs.name = AENEAS;
    } else if (rover_name == "ajax") {
        agent.name = AJAX;
//        rovers.agent_refs.name = AJAX;
    }
    std_msgs::String curr_pose = roverPose(message);
    currentPose.publish(curr_pose);
    // Place in rovers data in 'swarm'
//    swarm.vec[agent.name] = agent;
//    swarm[rovers.agent_refs.name] = rovers;
//    std::cout << "Rover name: " << rovers.agent_refs.name << std::endl;
//    std::cout << "Swarm rovers: " << swarm[rovers.agent_refs.name].agent_refs.name << std::endl;
//    std::cout << "Rover heading: " << rovers.agent_refs.current_pose.theta << std::endl;
//    std::cout << "Swarm rovers heading: " << swarm[rovers.agent_refs.name].agent_refs.current_pose.theta << std::endl;
    std::cout << "Rover name: " << agent.name << std::endl;
//    std::cout << "Swarm rovers: " << swarm.swarm_vector[agent.name].name << std::endl;
    std::cout << "Rover heading: " << agent.current_pose.theta << std::endl;
//    std::cout << "Swarm rovers heading: " << swarm.swarm_vector[agent.name].current_pose.theta << std::endl;
//    std_msgs::String global_avg_heading = globalHeading();
//    globalAverageHeading.publish(global_avg_heading);
    // Place update swarm_vector with gAH in 'swarm'
//    swarm[rovers.agent_refs.name] = rovers;
//    swarm.swarm_vector[agent.name] = agent;
//    std::cout << "Rover gAH: " << rovers.agent_refs.global_heading << std::endl;
//    std::cout << "Swarm gAH: " << swarm[rovers.agent_refs.name].agent_refs.global_heading << std::endl;
}

std_msgs::String roverPose (const nav_msgs::Odometry::ConstPtr &message) {
    char buf[256];
    std_msgs::String content;
    double curr_x = message->pose.pose.position.x;
    double curr_y = message->pose.pose.position.y;

    /*
     * Get theta rotation by converting quaternion orientation to pitch/roll/yaw
     */
    tf::Quaternion q(message->pose.pose.orientation.x, message->pose.pose.orientation.y,
                     message->pose.pose.orientation.z, message->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    snprintf(buf, 256, "Rover name: %s, x: %lf, y: %lf, theta: %lf",
             rover_name.c_str(), curr_x, curr_y, yaw);
    agent.current_pose.x = curr_x;
    agent.current_pose.y = curr_y;
    agent.current_pose.theta = yaw;
    content.data = string(buf);

    return content;
}
//
//std_msgs::String globalHeading (){
//    char buf[256];
//    std::vector<double> thetaG(2);
//    std_msgs::String content;
//    double gAH;
//    /*
//     * Dynamically average global heading
//     * NOTE: independent of the number of rovers
//     */
////    for (std::vector<ROVER_REFS>::iterator iter = swarm.begin(); iter != swarm.end(); ++iter){
////        thetaG.at(0) += std::cos(rover_refs.current_pose.theta);
////        thetaG.at(1) += std::sin(rover_refs.current_pose.theta);
////    }
////    for (int i = 0; i < thetaG.size(); i++){ // Will only ever be of size two since only compontents are (x, y)
////        thetaG.at(i) /= swarm.size();
////    }
////    gAH = std::atan2(thetaG.at(1), thetaG.at(0));
////    rover_refs.global_heading = gAH;
//
//    snprintf(buf, 256, "Global Heading: %lf", gAH);
//    content.data = string(buf);
//
//    return content;
//}