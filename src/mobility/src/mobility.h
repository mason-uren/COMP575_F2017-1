//
// Created by administrator on 10/10/17.
//

#ifndef PROJECT_MOBILITY_H
#define PROJECT_MOBILITY_H

#include <geometry_msgs/Pose2D.h>
#include "Swarm.h"
#include "RoverPose.h"
#include "std_msgs/Float64MultiArray.h"

#include "ZoneMap.h"
#include "AgentMap.h"
#include "Driveway.h"


#define NEIGH_DIST 2
#define TUNING_CONST 0.45

/*
 * ROVER_REFS
 */
//Swarm swarm(NUM_ROVERS);
//AGENT_REFS agent;


// Mobility Logic Functions
void setVelocity(double linearVel, double angularVel);

// OS Signal Handler
void sigintEventHandler(int signal);

//Publishers
ros::Publisher velocityPublish;
ros::Publisher stateMachinePublish;
ros::Publisher status_publisher;
ros::Publisher target_collected_publisher;
ros::Publisher angular_publisher;
ros::Publisher messagePublish;
ros::Publisher debug_publisher;
/*
 * Added Publishers
 */
ros::Publisher currentPose;
ros::Publisher globalAverageHeading;
ros::Publisher localAverageHeading;

//Subscribers
ros::Subscriber joySubscriber;
ros::Subscriber modeSubscriber;
ros::Subscriber targetSubscriber;
ros::Subscriber obstacleSubscriber;
ros::Subscriber odometrySubscriber;
ros::Subscriber messageSubscriber;
/*
 * Added subscibers
 */
ros::Subscriber headingSubscriber;

//Timers
ros::Timer stateMachineTimer;
ros::Timer publish_status_timer;
ros::Timer killSwitchTimer;

// Callback handlers
void joyCmdHandler(const geometry_msgs::Twist::ConstPtr &message);
void modeHandler(const std_msgs::UInt8::ConstPtr &message);
void targetHandler(const shared_messages::TagsImage::ConstPtr &tagInfo);
void obstacleHandler(const std_msgs::UInt8::ConstPtr &message); //
void odometryHandler(const nav_msgs::Odometry::ConstPtr &message);
void mobilityStateMachine(const ros::TimerEvent &);
void publishStatusTimerEventHandler(const ros::TimerEvent &event);
void killSwitchTimerEventHandler(const ros::TimerEvent &event);
void messageHandler(const std_msgs::String::ConstPtr &message);
/*
 * Added Handlers
 */
void headingHandler(const std_msgs::Float64MultiArray::ConstPtr &message);

/*
 * Helper Functions
 */
std_msgs::String globalHeading ();
void neighbors (int name);
std_msgs::String localHeading (int name);
std_msgs::String localPose (int name);

//typedef enum {
//    ACHILLES = 0, AENEAS, AJAX, DIOMEDES, HECTOR, PARIS
//} ROVER_NAME;

//typedef struct {
//    std::vector<double> neighbor;
//} NEIGHBORS;

//typedef struct rover_pose{
//    int name;
//    double x;
//    double y;
//    double theta;
//    NEIGHBORS neighbors;
//} ROVER_POSE;

std::map<int, RoverPose> rover_hash;
#endif //PROJECT_MOBILITY_H
