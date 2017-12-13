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
int rover;

int simulation_mode = 0;
float mobility_loop_time_step = 0.1;
float status_publish_interval = 5;
float kill_switch_timeout = 10;

geometry_msgs::Pose2D current_location;

int transitions_to_auto = 0;
double time_stamp_transition_to_auto = 0.0;

// state machine states
#define STATE_MACHINE_TRANSLATE 0
int state_machine_state = STATE_MACHINE_TRANSLATE;

bool check = false;
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
     * Determine the agent we are operating on
     */
    if (rover_name == "achilles") {
        std::cout << "ACHILLES" << std::endl;
        rover = ACHILLES;
    } else if (rover_name == "aeneas") {
        std::cout << "AENEAS" << std::endl;
        rover = AENEAS;
    } else if (rover_name == "ajax") {
        std::cout << "AJAX" << std::endl;
        rover = AJAX;
    } else if (rover_name == "diomedes") {
        std::cout << "DIOMEDES" << std::endl;
        rover = DIOMEDES;
    } else if (rover_name == "hector") {
        std::cout << "HECTOR" << std::endl;
        rover = HECTOR;
    } else if (rover_name == "paris") {
        std::cout << "PARIS" << std::endl;
        rover = PARIS;
    }

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

    /*
     * Deallocate memory for agent
     */
//    delete agent;

    ros::spin();
    return EXIT_SUCCESS;
}

void mobilityStateMachine(const ros::TimerEvent &)
{
    std::cout << "STATE MACHINE" << std::endl;
    std::cout << "MAP SIZE <== " << agentMap->getSize() << std::endl;
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
            /*
             * TODO: this is where we put the rover states code (implemenation that's written in local main.cpp)
             * NOTE: states need to be made into classes
             */
            double angle;
            double dist_toAnchor;
            GOAL_ZONE_POSE gz;
            VELOCITY vel = {0, 0};
            geometry_msgs::Pose2D anchor_pose;
            *agent = agentMap->getValue(rover);
            std::cout << "(ID) ---> " << agent->getID() << ", Rover <== " << rover << std::endl;
            int state = agent->getState();
            if (agent->getLocalization()->isAnchor()) {
                dist_toAnchor = agent->distFromAnchor();
            }
            switch (state) {
                case STATE_INIT: {
                    std::cout << "STATE -> INIT" << std::endl;

                    LOC_SUBSTATE init_sub = agent->getLocalization()->getSubstate();
                    std::cout << "STATE -> INIT (ID) ---> " << agent->getID() << ", Rover <== " << rover << std::endl;
                    std::cout << "STATE -> INIT (POSE) ---> {" << agent->getCurrPose().x << ", " << agent->getCurrPose().y << "}" << std::endl;
                    std::cout << "STATE -> INIT (SUBSTATE) ---> " << init_sub << std::endl;
                    switch (init_sub) {
                        /*
                         * Average the poses of the agents and set a global reference point
                         * 1) Calculate midpoint distance from other rovers and average MAX_ITER times
                         * 2) Average midpoints
                         * 3) Set anchor_node
                         */
                        case ANCHORING: {
                            std::cout << "SUBSTATE ---> ANCHORING" << std::endl;
                            int iter = agent->getLocalization()->getIter();
                            std::map<int, Agent> agentMap_copy = agentMap->getMapCopy();
                            double x_avg;
                            double y_avg;
                            if (iter >= 0 && iter < MAX_ITER) {
                                std::cout << "SUBSTATE ---> ANCHORING -> (1)" << std::endl;
                                /*
                                 * Make sure rover offset places rovers in correct positions for anchoring
                                 */
                                switch  (rover) {
                                    case ACHILLES: // Starting simulator offset (0,1)
                                        offset.x = 0 - current_location.x;
                                        offset.y = 1 - current_location.y;
                                        break;
                                    case AENEAS: // Starting simulator offest (-1,0)
                                        offset.x = -1 - current_location.x;
                                        offset.y = 0 - current_location.y;
                                        break;
                                    case AJAX: // Starting simulator offset (1,0)
                                        offset.x = 1 - current_location.x;
                                        offset.y = 0 - current_location.y;
                                        break;
                                    case DIOMEDES: // Starting simulator offset (1,1)
                                        offset.x = 1 - current_location.x;
                                        offset.y = 1 - current_location.y;
                                        break;
                                    case HECTOR: // Startign simulator offset (-1,-1)
                                        offset.x = -1 - current_location.x;
                                        offset.y = -1 - current_location.y;
                                        break;
                                    case PARIS: // Startign simulator offset (1, -1)
                                        offset.x = 1 - current_location.x;
                                        offset.y = -1 - current_location.y;
                                        break;
                                    default:
                                        std::cout << "ERROR: there are more agents than specificed ()." << std::endl;
                                        break;
                                }
                                current_location.x += offset.x;
                                current_location.y += offset.y;
                                agent->setCurrPose(current_location);
                                agentMap->updateMap(rover, *agent);
                                // Iterate through all agents except self
                                for (std::map<int, Agent>::iterator it = agentMap_copy.begin();
                                     it != agentMap_copy.end(); ++it) {
                                    if (it->first != agent->getID()) { // We want the midpoints
                                        x_avg += it->second.getCurrPose().x / 2;
                                        y_avg += it->second.getCurrPose().y / 2;
                                    }
                                }
                                x_avg /= agentMap_copy.size();
                                y_avg /= agentMap_copy.size();
                                agent->getLocalization()->addMidpoint(x_avg, y_avg);
                                agent->getLocalization()->incrmtIter();
                                agentMap->updateMap(rover, *agent);
                            }
                            /*
                             * Average the midpoints into an estimated pose
                             */
                            else if (iter == MAX_ITER) {
                                std::cout << "SUBSTATE ---> ANCHORING -> (2)" << std::endl;
                                std::vector<geometry_msgs::Pose2D> midpoints = agentMap->getValue(rover).getLocalization()->getMidpoints();
                                for (std::vector<geometry_msgs::Pose2D>::iterator it = midpoints.begin();
                                     it != midpoints.end(); ++it) {
                                    x_avg += it->x;
                                    y_avg += it->y;
                                }
                                x_avg /= midpoints.size();
                                y_avg /= midpoints.size();
                                agent->getLocalization()->setEstimated(x_avg, y_avg);
                                std::cout << "Estimated Pose = {" << agent->getLocalization()->getEstimated().x << ", " << agent->getLocalization()->getEstimated().y << "}" << std::endl;
                                agent->getLocalization()->incrmtIter();
                                agentMap->updateMap(rover, *agent);
                            }
                            /*
                             * Set anchor pose
                             */
                            else if (iter == MAX_ITER + 1) {
                                std::cout << "SUBSTATE ---> ANCHORING -> (3)" << std::endl;
                                for (std::map<int, Agent>::iterator it = agentMap_copy.begin(); it != agentMap_copy.end(); ++it) {
                                    x_avg += it->second.getLocalization()->getEstimated().x;
                                    y_avg += it->second.getLocalization()->getEstimated().y;
                                }
                                x_avg /= agentMap_copy.size();
                                y_avg /= agentMap_copy.size();
                                std::cout << "ANCHORING ---> ANCHOR NODE {" << x_avg << ", " << y_avg << "}" << std::endl;
                                agent->getLocalization()->setAnchor(x_avg, y_avg);
                                agent->getLocalization()->advanceSubstate();
                                agent->getLocalization()->incrmtIter();
                                agentMap->updateMap(rover, *agent);
                            }
                            break;
                        }
                        /*
                         * Set confidence interval based around how far away the current rover is from the new anchor_node
                         * NOTE: this should be roughly the same value for each agent
                         * 1) Set confidence w.r.t. to anchor
                         * 2) Advance substate
                         * 3) Change agent state
                         */
                        case WEIGHTING:
                            std::cout << "SUBSTATE ---> WEIGHTING" << std::endl;
                            agent->getLocalization()->setAnchConfidence(dist_toAnchor);
                            agent->getLocalization()->advanceSubstate();
                            agent->getLocalization()->resetIter();
                            agent->getLocalization()->setSubstate(BEGINNING);
                            agent->setState(STATE_SEARCH);
                            agentMap->updateMap(rover, *agent);
                            break;
                        default:
                            break;
                    }
                    std::cout << "Incriment" << std::endl;
                    std::cout << "ITER <== " << agent->getLocalization()->getIter() << std::endl;
                    break;
                }
                /*
                 * Drive rovers in outward radial pattern from their initial positions
                 */
                case STATE_SEARCH: {
                    std::cout << "STATE -> SEARCH" << std::endl;
                    std::cout << "STATE -> SEARCH ---> CURRENT LOCATION {" << agent->getCurrPose().x << ", " << agent->getCurrPose().y << "}" << std::endl;
                    if (agent->getLocalization()->getIter() == 0) {
                        search_pose.x = agent->getCurrPose().x * 3;
                        search_pose.y = agent->getCurrPose().y * 3;
                        agent->setGoalPose(search_pose);
                        double dist_toGoal = agent->distFromGoal();
                        agent->getLocalization()->setGoalConfidence(dist_toGoal);
                        agent->getLocalization()->incrmtIter();
                        agentMap->updateMap(rover, *agent);
                    }
                    angle = zoneMap->angleCalc(agent->getGoalPose(), agent->getCurrPose());
                    angle = angles::shortest_angular_distance(angle, agent->getCurrPose().theta);
                    double dist = tangentialDist(agent->getGoalPose(), current_location);
                    if (dist < 0.5) {
                        agent->setAngVel(0);
                        agent->setLinVel(0);
                    }
                    else {
                        agent->setAngVel(0.1 * -angle);
                        agent->setLinVel(0.02 * dist);
                    }
                    agent->getLocalization()->setVelConfidence(agent->getLinVel());
                    agentMap->updateMap(rover, *agent);
                    std::cout << "Search Pose: X <- " << agent->getGoalPose().x << ", Y <- " << agent->getGoalPose().y << std::endl;
                    std::cout << "Dist to goal: " << dist << std::endl;
                    std::cout << "Angle to goal: " << angle << std::endl;
                    break;
                }
                case STATE_PICK_UP:
                    break;
                case STATE_FIND_HOME: {
                    gz = agent->getGZPose();
                    switch (agent->getDrivewayState()) {
                        case INIT: {
                            if (driveway->canEnter((DRIVEWAY_TYPE) agent->getDrivewayState())) {
                                *agent = driveway->addToDriveway(*agent, ACTIVE);
//                                agentMap->updateMap(agent->getID(), *agent);
                            } else {
                                *agent = driveway->addToDriveway(*agent, WAITING);
//                                agentMap->updateMap(agent->getID(), *agent);
                            }
                            vel.linear = 0;
                            vel.angular = 0;
                            break;
                        }
                        case ACTIVE: {
                            // Determine closest zone
                            zoneMap->checkAvailable();
                            *agent = zoneMap->getClosestZone(*agent);
                            agentMap->updateMap(agent->getID(), *agent); // Update map that claims zone
                            *agent = agentMap->getValue(agent->getID());

                            // Grab critical points for zone
                            CriticalPoints cps = innerRadius->getCP(agent->getGZPose().zone_ID);
                            double dist_toZone = tangentialDist(gz.goal_pose, current_location);
                            angle = zoneMap->angleCalc(gz.goal_pose, current_location);
                            /*
                             * If goal zone is not right in front of the agent
                             */
                            if (gz.traverse) {
                                double factor;
                                double dist_cpL = tangentialDist(cps.leftCP(), current_location);
                                /*
                                 * Check if critical point along traversal are near
                                 * NOTE: I can't remember why I chose to create and check 'getReachedCPS' function
                                 */
                                if (dist_cpL < 0.2 || agent->getReachedCPS()) {
                                    /*
                                     * Change traversal path to traverse to zone garage
                                     * We need to calc equation of line depicted by critical point and where it meets RZ
                                     */
                                    // Are we at the goal zone
                                    if (dist_cpL < 0.1) { // TODO: arbitrary value
                                        *agent = driveway->moveAgent(*agent, ACTIVE, GARAGE);
                                        agentMap->updateMap(agent->getID(), *agent);
                                    } else if (current_location.theta <
                                               angle - TRAVERSE_ERR) { // Left turn; considering angles
                                        angle = 0.025 / dist_toAnchor; // TODO: arbitrary value
                                    } else if (current_location.theta >
                                               angle + TRAVERSE_ERR) { // Right turn; considering angles
                                        angle = -0.025 / dist_toAnchor; // TODO: arbitrary value
                                    } else {
                                        // Do nothing continue driving forward
                                        angle = current_location.theta;
                                    }
                                    vel.linear = 0.2 * (dist_toZone - +(RZ / 2)); // TODO: arbitrary value
                                } else {
                                    /*
                                     * Default Traversal
                                     */
                                    // Drive till we are in the middle of the driveway
                                    if (!agent->getInitTraversal()) {
                                        if (dist_toAnchor > CPR) {
                                            agent->setInitTraversal(true);
                                            agentMap->updateMap(agent->getID(), *agent);
                                        }
                                        angle = -0.2 / dist_toAnchor; // TODO: arbitrary value
                                    } else if (dist_toAnchor <= CPR - TRAVERSE_STD) {
                                        factor = CPR - TRAVERSE_STD - dist_toAnchor;
                                        angle = (-0.1 / dist_toAnchor) * factor; // TODO: arbitrary value
                                    } else if (dist_toAnchor >= CPR + TRAVERSE_STD) {
                                        factor = dist_toAnchor - CPR - TRAVERSE_ERR;
                                        angle = (0.1 / dist_toAnchor) * factor; // TODO: arbitrary value
                                    } else {
                                        angle = 0.1 / dist_toAnchor; // Need to continually turn toward goal poze
                                    }
                                    vel.linear = 0.2 * dist_toZone; // TODO: arbitrary value
                                }
                            }
                                /*
                                 * Goal zone is right in front
                                 */
                            else {
                                if (dist_toZone < 0.1) {
                                    *agent = driveway->moveAgent(*agent, ACTIVE, GARAGE);
                                    agentMap->updateMap(agent->getID(), *agent);
                                }
                                if (current_location.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                                    angle = 0.025 / dist_toAnchor; // TODO: arbitrary value
                                } else if (current_location.theta >
                                           angle + TRAVERSE_ERR) { // Right turn; considering angles
                                    angle = -0.025 / dist_toAnchor; // TODO: arbitrary value
                                } else {
                                    // Do nothing continue driving forward
                                    angle = current_location.theta;
                                }
                                vel.linear = 0.2 * (dist_toZone + (RZ / 2)); // TODO: arbitrary value
                            }
                            vel.angular = angle;
                            break;
                        }
                            /*
                             * Check if the rover is allowed to enter the driveway
                             */
                        case WAITING: {
                            if (driveway->canEnter((DRIVEWAY_TYPE) agent->getDrivewayState())) {
                                *agent = driveway->moveAgent(*agent, WAITING, ACTIVE);
                                agentMap->updateMap(agent->getID(), *agent);
                            }
                            break;
                        }
                            /*
                             * We need to turn the rover to face home
                             */
                        case GARAGE: {
                            angle = zoneMap->angleCalc(gz.goal_pose, current_location);
                            if (current_location.theta < angle - ORIENTATION_ERR) { // Left turn; considering angles
                                angle = 0.2 / dist_toAnchor; // TODO: arbitrary value
                            } else if (current_location.theta >
                                       angle + ORIENTATION_ERR) { // Right turn; considering angles
                                angle = -0.2 / dist_toAnchor; // TODO: arbitrary value
                            } else {
                                if (driveway->canEnter((DRIVEWAY_TYPE) agent->getDrivewayState())) {
                                    *agent = driveway->moveAgent(*agent, GARAGE, DELIVERY);
                                    agentMap->updateMap(agent->getID(), *agent);
                                }
                            }
                            vel.linear = 0.001;
                            vel.angular = angle;
                            break;
                        }
                        case DELIVERY: {
                            /*
                             * The agent has reached the drop off point
                             * 1) Transition to DROP_OFF, which should have no properties except to drop the cube
                             * 2) DROP_OFF passes back after cube is released, but no movement is to be made by DROP_OFF,
                             *      DROP_OFF needs to mark that the agent no longer has a resource
                             */
                            angle = zoneMap->angleCalc(gz.goal_pose, current_location);
                            if (dist_toAnchor < 0.2) {
                                vel.linear = 0.001;
                                angle = current_location.theta;
                                // Pass to DROP_OFF
                                agent->setState(STATE_DROP_OFF);
                                agentMap->updateMap(agent->getID(), *agent);
                            } else {
                                if (current_location.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                                    angle = 0.05 / dist_toAnchor; // TODO: arbitrary value
                                } else if (current_location.theta >
                                           angle + TRAVERSE_ERR) { // Right turn; considering angles
                                    angle = -0.05 / dist_toAnchor; // TODO: arbitrary value
                                } else {
                                    // Do nothing continue driving forward
                                    angle = current_location.theta;
                                }
                                vel.linear = 0.2 * dist_toAnchor;
                            }
                            vel.angular = angle;
                            break;
                        }
                        default:
                            break;
                    }
                    break;
                }
                case STATE_OBSTACLE_AVOIDANCE:
                    break;
                case STATE_DROP_OFF:
                    // Place cube or equivalent of that in this implementation
                    agent->setState(STATE_LEAVE_HOME);
                    agent->setResource(false);
                    agentMap->updateMap(agent->getID(), *agent);
                    break;
                case STATE_LEAVE_HOME: {
                    gz = agent->getGZPose();
                    angle = zoneMap->angleCalc(gz.goal_pose, current_location);
                    if (dist_toAnchor >
                        R3 + TRAVERSE_ERR) { // EXIT state <--- need to wrap up the code into different states
                        agent->setInitTraversal(false);
                        agent->setState(STATE_SEARCH);
                        agentMap->updateMap(agent->getID(), *agent);
                    } else if (current_location.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                        angle = 0.025 / dist_toAnchor;
                    } else if (current_location.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
                        angle = -0.025 / dist_toAnchor;
                    }
                    vel.linear = -0.2 * dist_toAnchor;
                    vel.angular = angle;
                    break;
                }
                default:
                    std::cout << "ERROR: bad agent state." << std::endl;
                    break;
            }
            agentMap->updateMap(rover, *agent);



            state_machine_msg.data = "TRANSLATING";//, " + converter.str();
//            double angular_velocity = TUNING_CONST * (agentMap->getValue(rover).getGlobalTheta() - current_location.theta);
//            double angular_velocity = TUNING_CONST * (rover_hash[rover].avg_local_theta - current_location.theta);
//            double angular_velocity = TUNING_CONST * (rover_hash[rover].avg_local_pose - current_location.theta);
//            float linear_velocity = 0;
            /*
             * Testing velocities
             */
            double angular_velocity = agent->getAngVel();
            std::cout << "Angular velecity: " << angular_velocity << std::endl;
            double linear_velocity = agent->getLinVel();
            std::cout << "Linear velocity: " << linear_velocity << std::endl;
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
//    std::cout << "CURRENT LOCATION ---> {" << agent->getCurrPose().x << ", " << agent->getCurrPose().y << "}" << std::endl;
    std::cout << "End publisher" << std::endl;


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
    double perceived_pose[] = {message->pose.pose.position.x, message->pose.pose.position.y};
    current_location.x = perceived_pose[0];
    current_location.y = perceived_pose[1];

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
void headingHandler(const std_msgs::Float64MultiArray::ConstPtr &message) { // TODO: implement new frame to test to see if it works with old functionality
    int current_rover = (int) message->data[0];
    geometry_msgs::Pose2D msg_pose;
    msg_pose.x = message->data[1];
    msg_pose.y = message->data[2];
    msg_pose.theta = message->data[3];

    // Create Agent
    if (!agentMap->exists(current_rover)){
        agent = new Agent (current_rover, msg_pose);
        agentMap->addToMap(current_rover, *agent);
    }
    else {
        *agent = agentMap->getValue(current_rover);
        double x_off;
        double y_off;
        if (agent->getState() != STATE_INIT && agent->getLocalization()->getIter()) {
            double angle = std::atan2(agent->getGoalPose().y - msg_pose.y, agent->getGoalPose().x - msg_pose.x);
            // GPS ERROR
            agent->getLocalization()->setAnchConfidence(agent->distFromAnchor());
            // Odometry Error
            agent->getLocalization()->setGoalConfidence(agent->distFromGoal());
            // Dynamic localization
            agent->getLocalization()->setVelConfidence(agent->getLinVel());
            double w = agent->getLocalization()->getAnchConfidence();
            double u = agent->getLocalization()->getGoalConfidence();
            double v = agent->getLocalization()->getVelConfidence();
            x_off = w * (agent->getCurrPose().x - agent->getPrevPose().x) + u * (agent->getGoalPose().x - agent->getCurrPose().x) + v * std::cos(angle);
            y_off = w * (agent->getCurrPose().y - agent->getPrevPose().y) + u * (agent->getGoalPose().y - agent->getCurrPose().y) + v * std::sin(angle);
            msg_pose.x += x_off;
            msg_pose.y += y_off;
            agent->setCurrPose(msg_pose);
            agentMap->updateMap(current_rover, *agent);
        }
    }







//    Call globalHeading
    std_msgs::String gH = globalHeading();
    globalAverageHeading.publish(gH);
    char *end;
    agent->setGlobalTheta(std::strtod(gH.data.c_str(), &end));
    agentMap->updateMap(agent->getID(), *agent);
    end = NULL; // clear pointer

//    Call neighbors
    neighbors(current_rover);


//    Call localHeading
//    std_msgs::String lH = localHeading(current_rover);
//    localAverageHeading.publish(lH);
//    agentMap->getValue(current_rover).setLocalTheta(std::strtod(lH.data.c_str(), &end));
//    end = NULL; // clear pointer

//    Call localPose
//    std_msgs::String lP = localPose(current_rover);
//    agentMap->getValue(current_rover).setLocalPose(std::strtod(lP.data.c_str(), &end));
//    end = NULL; // clear pointers

}

double goalCalc(Agent agent) {

}

std_msgs::String globalHeading () {
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double gAH;
    // Iterate through map an populate 'thetaG' with theta values
    std::map<int, Agent> agentMap_copy = agentMap->getMapCopy();
    for (std::map<int, Agent>::iterator it = agentMap_copy.begin(); it != agentMap_copy.end(); ++it) {
        thetaG.at(0) += cos(it->second.getCurrPose().theta);
        thetaG.at(1) += sin(it->second.getCurrPose().theta);
    }
    thetaG.at(0) /= thetaG.size();
    thetaG.at(1) /= thetaG.size();
    gAH = std::atan2(thetaG[1], thetaG[0]);

//    Format string
    snprintf(buf, 256, "%lf", gAH);
    content.data = string(buf);
    memset(buf, 0, 255); // Clear char array

    return content;
}


void neighbors (int name) {
    // Make a copy of the map
    std::map<int, Agent> agentMap_copy = agentMap->getMapCopy();
//    std::vector<int> nbr_vec;
    for (std::map<int, Agent>::iterator it = agentMap_copy.begin(); it !=agentMap_copy.end(); ++it) {
        agentMap_copy[it->first].clearNeighbors();
    }
    // Hold constant the current rover
    Agent curr_rover = agentMap->getValue(name);
    // Iterate through remaining rovers
    for (std::map<int, Agent>::iterator it = agentMap_copy.begin(); it != agentMap_copy.end(); ++it) {
        if ((it->first != name) && (hypot(curr_rover.getCurrPose().x - it->second.getCurrPose().x, curr_rover.getCurrPose().y - it->second.getCurrPose().y) < NEIGH_DIST)) {
            agentMap_copy[name].addNeighbors(it->first); // Save the neighbors to the constant rover
            agentMap_copy[it->first].addNeighbors(name); // Save the neighbors to the other rover
        }
    }
    agentMap->updateEntireMap(agentMap_copy);
}


std_msgs::String localHeading (int name) {
    std::cout << "LOCAL HEADING" << std::endl;
    char buf[256];
    static const int arr[] = {0,0}; // Default bad values
    std::vector<double> thetaG(arr, arr + sizeof(arr) / sizeof(arr[0]));
    std_msgs::String content;
    double lAH;
    std::vector<double> rNeighb;

    // Iteratte through respective neighbors vector
    std::map<int, Agent> agentMap_copy = agentMap->getMapCopy();
    std::vector<int> vec = agentMap_copy[name].getNeighbors();
    for (std::vector<int>::iterator it = vec.begin(); it != vec.end(); ++it) {
        std::cout << "Neighbors " << *it << std::endl;
        rNeighb.push_back(*it);
    }
    // Iterator through neighboring rovers pose
    for (std::vector<double>::iterator it = rNeighb.begin(); it != rNeighb.end(); ++it){
        thetaG.at(0) += cos(agentMap_copy[*it].getCurrPose().theta);
        thetaG.at(1) += sin(agentMap_copy[*it].getCurrPose().theta);
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

    // Iterate through respective neighbors vector
    std::map<int, Agent> agentMap_copy = agentMap->getMapCopy();
    for (std::vector<int>::iterator it = agentMap_copy[name].getNeighbors().begin(); it != agentMap_copy[name].getNeighbors().end(); ++it) {
        rNeighb.push_back(*it);
    }
    std::cout << "After first loop" << std::endl;
    // Iterate through neighboring rovers pose
    for (std::vector<double>::iterator it = rNeighb.begin(); it != rNeighb.end(); ++it) {
        thetaG.at(0) += agentMap_copy[*it].getCurrPose().x;
        thetaG.at(1) += agentMap_copy[*it].getCurrPose().y;
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