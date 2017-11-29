#include <iostream>

#include "main.h"
#include "Zone.h"
#include "InnerRadius.h"
#include "Localization.h"

using namespace std;

// (X, Y, THETA)
POSE rover1 = (POSE) {-std::cos(R2), std::sin(R2), 0};
POSE rover2 = (POSE) {std::cos(R2), std::sin(R2), 0};
POSE rover3 = (POSE) {R2, 0, 0};
POSE rover4 = (POSE) {std::cos(R2), -std::sin(R2), 0};
POSE rover5 = (POSE) {-std::cos(R2), -std::sin(R2), 0};
POSE rover6 = (POSE) {-R2, 0, 0};

POSE poseGenerator (POSE pose) {
    POSE generated;
    double minX = pose.x - LOCALIZATION_ERROR;
    double maxX = pose.x + LOCALIZATION_ERROR;
    double minY = pose.y - LOCALIZATION_ERROR;
    double maxY = pose.y + LOCALIZATION_ERROR;
    int Xmin = (int) minX * 10000;
    int Xmax = (int) maxX * 10000;
    int Ymin = (int) minY * 10000;
    int Ymax = (int) maxY * 10000;
    while (true) {
        generated.x = std::rand() % (Xmax - Xmin) + Xmin;
        generated.x /= 10000;

        generated.y = std::rand() % (Ymax - Ymin) + Ymin;
        generated.y /= 10000;

        generated.theta = 0;

        if (tangentialDist(generated, pose) < LOCALIZATION_ERROR) {
            break;
        }
    }
    return generated;
}

int main() {
    std::cout << "Hello, World!" << std::endl;

    typedef struct {
        double angular;
        double linear;
    } VELOCITY;

    // Create ZoneMap
    ZoneMap<std::string, Zone> *zoneMap = new ZoneMap<std::string, Zone>();
    AgentMap<int, Agent> *agentMap = new AgentMap<int, Agent>();
    Driveway<int> *driveway = new Driveway<int>();

    // Add agents (Setup for STATE_INIT where every is around home)
    agentMap->addToMap(ACHILLES, Agent::Agent(ACHILLES, poseGenerator(rover1))); // 1
    agentMap->addToMap(MASON, Agent::Agent(MASON, poseGenerator(rover2))); // 2
    agentMap->addToMap(AJAX, Agent::Agent(AJAX, poseGenerator(rover3))); // 3
    agentMap->addToMap(TURD_SANDO, Agent::Agent(TURD_SANDO, poseGenerator(rover4))); // 4
    agentMap->addToMap(AENEUS, Agent::Agent(AENEUS, poseGenerator(rover5))); // 5
    agentMap->addToMap(PETER, Agent::Agent(PETER, poseGenerator(rover6))); // 6

    // Change states
    Agent agent;
//    agent = agentMap->getValue(ACHILLES);
//    agent.getState()->setState(STATE_FIND_HOME);
//    agentMap->updateMap(agent.getID(), agent);
//
//    agent = agentMap->getValue(MASON);
//    agent.getState()->setState(STATE_OBSTACLE_AVOIDANCE);
//    agentMap->updateMap(agent.getID(), agent);
//
//    agent = agentMap->getValue(AJAX);
//    agent.getState()->setState(STATE_SEARCH);
//    agentMap->updateMap(agent.getID(), agent);

    // Setup variables
    VELOCITY velocity;
    agent = agentMap->getValue(ACHILLES); // TODO: need to make sure this value varies between each rover (ie ACHILLES need to be dynamic)
    double x;
    double y;
    double angle;
    GOAL_ZONE_POSE gz;
    POSE curr_pose = agent.getCurrPose();;
    POSE anchor_pose = agent.getLocalization()->getAnchor();
    double dist_anchor = agent.distFromAnchor();
    int state = agent.getState()->getState();

    switch (state) {
        case STATE_INIT: {
            /*
             * LOCALIZATION
             * TODO: The rovers each think their initial pose is 0,0; we need to create a way to adjust their pose inorder to mark anchor node
             *
             */
            // Inititialization may have to be done in an INIT_CLASS
            int iter = agent.getLocalization()->getInit().iterations;
            std::map<int, Agent> copy_map = agentMap->getMapCopy();
            double x_avg = 0;
            double y_avg = 0;
            if (iter >= 0 && iter < MAX_ITER) {
                // Iterate through all agent except yourself
                for (std::map<int, Agent>::iterator it = copy_map.begin(); it != copy_map.end(); ++it){
                    if (it->first != agent.getID()){ // We want the mid points
                        x_avg += it->second.getCurrPose().x / 2;
                        y_avg += it->second.getCurrPose().y / 2;
                    }
                }
                x_avg /= copy_map.size() - 1;
                y_avg /= copy_map.size() - 1;
                iter++;
                // Save averaged anchor pose
                agent.getLocalization()->addMidpoint(x_avg, y_avg);
                agent.getLocalization()->setIter(iter);
                agentMap->updateMap(agent.getID(), agent);
            }
            /*
             * Average midpoints into an estimated pose
             */
            else if (iter == MAX_ITER) {
                std::vector<POSE> midpoints = agent.getLocalization()->getMidpoints();
                for (std::vector<POSE>::iterator it = midpoints.begin(); it != midpoints.end(); ++it) {
                    x_avg += it->x;
                    y_avg += it->y;
                }
                x_avg /= midpoints.size();
                y_avg /= midpoints.size();
                iter++;
                agent.getLocalization()->setIter(iter);
                agent.getLocalization()->setEstimated(x_avg, y_avg);
                agentMap->updateMap(agent.getID(), agent);
            }
            /*
             * Set anchor pose and assign weight TODO: need to assign weight
             */
            else if (iter == MAX_ITER + 1) {
                for (std::map<int, Agent>::iterator it = copy_map.begin(); it != copy_map.end(); ++it) {
                    x_avg += it->second.getLocalization()->getEstimated().x;
                    y_avg += it->second.getLocalization()->getEstimated().y;
                }
                x_avg /= copy_map.size();
                y_avg /= copy_map.size();
                iter++;
                agent.getLocalization()->setAnchor(x_avg, y_avg);
                agent.getLocalization()->setIter(iter);
                agentMap->updateMap(agent.getID(), agent);
            }
            break;
        }
        case STATE_SEARCH: {
            /*
                * Found the threshold of driveway but doesn't not have a resource.
                * NEED: to traverse outer boundary (R3) and make decision when to leave boundary
                * TODO: '0.2' is an arbitrary value and needs to be set,
                */
            // TODO: need to figure out exit point; need to calculate when 'goal_search_waypoint' has a clear path
            // TODO: exit point sets 'initTraversal' back to zero
            if (!agent.getInitTraversal()) {
                if (dist_anchor > R3) { // INIT: this is the initial right turn to orient agent into traversal
                    // We need some linear momentum otherwise we will never clear this condition for traversal
                    agent.setInitTraversal(true);
                    agentMap->updateMap(agent.getID(), agent);

                }
                angle = -0.2 / dist_anchor;
            } else if (dist_anchor > (R3 + (3 * TRAVERSE_ERR))) {
                angle = 0.2 / dist_anchor;
            } else if (dist_anchor < (R3 + TRAVERSE_ERR)) {
                // Needs to turn right to begin outer traversal
                // We make a right turn by holding the linear velocity constant
                angle = -0.2 / dist_anchor;
            } else {
                // Drive straight
                angle = curr_pose.theta;
            }
            velocity.linear = 0.2 * dist_anchor; // TODO: '0.2' arbitrary
            velocity.angular = angle;
            break;
        }
        case STATE_PICK_UP:
            break;
        case STATE_FIND_HOME: {
            gz = agent.getGZPose();
            switch (agent.getDrivewayState()) {
                case INIT: {
                    if (driveway->canEnter((DRIVEWAY_TYPE) agent.getDrivewayState())) {
                        agent = driveway->addToDriveway(agent, ACTIVE);
                        agentMap->updateMap(agent.getID(), agent);
                    } else {
                        agent = driveway->addToDriveway(agent, WAITING);
                        agentMap->updateMap(agent.getID(), agent);
                    }
                    velocity.linear = 0;
                    velocity.angular = 0;
                    break;
                }
                case ACTIVE: {
                    // Determine closest zone
                    zoneMap->checkAvailable();
                    agent = zoneMap->getClosestZone(agent);
                    agentMap->updateMap(agent.getID(),
                                        agent); // Update map that claims zone -> need to update agentMap eachtime agent variable is changed
                    agent = agentMap->getValue(agent.getID());

                    // Grab critical points for zone
                    CRITICAL_POINTS cps = driveway->getCP(agent.getGZPose().zone_ID); // Grab ID of selected goal zone
                    double dist_zone = tangentialDist(gz.goal_pose, curr_pose);
                    angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
                    /*
                     * If goal zone is not right in front of the agent
                     * TODO: 'waypoint.x' and 'waypoint.y' may need to be set to 'curr_pose'
                     */
                    if (gz.traverse) {
                        double factor;
                        double dist_cpl = tangentialDist(cps.left_CP, curr_pose);
                        // Check if critical points along traversal are near
                        // NOTE: I can't remember why I chose to create and check 'getReachedCPS' function
                        if (dist_cpl < 0.2 || agent.getReachedCPS()) { // Arbitrary value; NOTE: needs to large enough to capture rover traversing away from R2
                            /*
                             * Change traversal path to traverse to zone garage
                             * We need to calc equation of line depicated by critical point and where it meets RZ
                             */
                            // Are we at the goal zone pose
                            if (dist_zone < 0.1) { // TODO: arbitrary value; will need to be tuned
                                agent = driveway->moveAgent(agent, ACTIVE, GARAGE);
                                agentMap->updateMap(agent.getID(), agent);
                            } else if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                                angle = 0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
                            } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
                                angle = -0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
                            } else {
                                // Do nothing continue driving forward
                                angle = curr_pose.theta;
                            }
                            velocity.linear = 0.2 * (dist_zone + (RZ / 2));
                        } else {
                            /*
                             * Default traversal:
                             * Drive till we are in the middle of driveway
                             */
                            if (!agent.getInitTraversal) {
                                if (dist_anchor > CPR) {
                                    agent.setInitTraversal(true);
                                    agentMap->updateMap(agent.getID(), agent);
                                }
                                angle = -0.2 / dist_anchor;
                            } else if (dist_anchor <= CPR - TRAVERSE_STD) { // Need to turn right to remain on driveway
                                factor = CPR - TRAVERSE_STD - dist_anchor;
                                angle = (-0.1 / dist_anchor) * factor; // TODO: arbitrary value; will need to be tuned

                            } else if (dist_anchor >= CPR + TRAVERSE_STD) { // Need to left to remain on driveway
                                factor = dist_anchor - CPR - TRAVERSE_STD;
                                angle = (0.1 / dist_anchor) * factor; // TODO: arbitrary value; will need to be tuned
                            } else {
                                angle = 0.1 / dist_anchor; // Need to continually turn toward goal poze
                            }
                            velocity.linear = 0.2 * (dist_zone); // TODO: arbitrary value; will need to be tuned
                        }

                    } else { // Goal zone is right in front
                        if (dist_zone < 0.1) {
                            agent = driveway->moveAgent(agent, ACTIVE, GARAGE);
                            agentMap->updateMap(agent.getID(), agent);
                        }
                        if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                            angle = 0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
                        } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
                            angle = -0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
                        } else {
                            // Do nothing continue driving forward
                            angle = curr_pose.theta;
                        }
                        velocity.linear = 0.2 * (dist_zone + (RZ / 2));
                    }
                    velocity.angular = angle;
                    break;
                }
                case WAITING: {
                    if (driveway->canEnter((DRIVEWAY_TYPE) agent.getDrivewayState())) {
                        agent = driveway->moveAgent(agent, WAITING, ACTIVE);
                        agentMap->updateMap(agent.getID(), agent);
                    }
                    velocity.linear = 0;
                    velocity.angular = 0;
                    break;
                }
                    // We need to make sure that even thought the rover is changing from GARAGE to DELIVERY the driveway still needs
                    // to mark that particular garage as held
                    // UPDATE: this is all handled in zoneMap since driveway and zoneMap are considered separately
                case GARAGE:
                    // We need to turn the rover to face home
                    angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
                    if (curr_pose.theta < angle - ORIENTATION_ERR) { // Left turn; considering angles
                        angle = 0.2 / dist_anchor; // TODO: arbitrary value; will need to be tuned
                    } else if (curr_pose.theta > angle + ORIENTATION_ERR) { // Right turn; considering angles
                        angle = -0.2 / dist_anchor; // TODO: arbitrary value; will need to be tuned
                    } else {
                        if (driveway->canEnter((DRIVEWAY_TYPE) agent.getDrivewayState())) {
                            agent = driveway->moveAgent(agent, GARAGE, DELIVERY);
                            agentMap->updateMap(agent.getID(), agent);
                        }
                    }
                    velocity.linear = 0.001;
                    velocity.angular = angle;
                    break;
                case DELIVERY: { // Traverse into home
                    /*
                    * The agent has reached the drop off point
                    * 1) Transition to DROP_OFF, which should have no properties accept to drop the cube
                    * 2) DROP_OFF passes back to after cube is released, but no movement is to be made by DROP_OFF.
                    *      DROP_OFF needs to mark that the agent no longer has a resource.
                    */
                    angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
                    if (dist_anchor < 0.2) {
                        velocity.linear = 0.001;
                        angle = curr_pose.theta;
                        // Pass to DROP_OFF
                        agent.getState()->setState(STATE_DROP_OFF);
                        agentMap->updateMap(agent.getID(), agent);
                    } else {
                        if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                            angle = 0.05 / dist_anchor;
                        } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
                            angle = -0.05 / dist_anchor;
                        } else {
                            // Do nothing continue driving forward
                            angle = curr_pose.theta;
                        }
                        velocity.linear = 0.2 * dist_anchor; // TODO: arbitrary value; will need to be tuned
                    }
                    velocity.angular = angle;
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
            agent.getState()->setState(STATE_LEAVE_HOME);
            break;
        case STATE_LEAVE_HOME: {
            angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
            if (dist_anchor > R3 + TRAVERSE_ERR) { // EXIT state <-- Need to wrap up code into different agent states
                agent.setInitTraversal(false);
                agent.getState()->setState(STATE_SEARCH);
                agentMap->updateMap(agent.getID(), agent);
            } else if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
                angle = 0.025 / dist_anchor;
            } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
                angle = -0.025 / dist_anchor;
            }
            velocity.linear = -0.2 * dist_anchor;
            velocity.angular = angle;
            break;
        }
        default:
            break;
    }




    /*
     * DRIVEWAY AND ZONES
     * We have a resource and we've just encountered the driveway
     *
     */
    // NEED: check that agent has resource and the outer threshold of driveway has been encountered
//    if (agent.getResource() && (dist_anchor < R3)) {
//        gz = agent.getGZPose();
//        switch (agent.getDrivewayState()) {
//            case INIT: {
//                if (driveway->canEnter((DRIVEWAY_TYPE) agent.getDrivewayState())) {
//                    agent = driveway->addToDriveway(agent, ACTIVE);
//                    agentMap->updateMap(agent.getID(), agent);
//                } else {
//                    agent = driveway->addToDriveway(agent, WAITING);
//                    agentMap->updateMap(agent.getID(), agent);
//                }
//                velocity.linear = 0;
//                velocity.angular = 0;
//                break;
//            }
//            case ACTIVE: {
//                // Determine closest zone
//                zoneMap->checkAvailable();
//                agent = zoneMap->getClosestZone(agent);
//                agentMap->updateMap(agent.getID(), agent); // Update map that claims zone -> need to update agentMap eachtime agent variable is changed
//                agent = agentMap->getValue(agent.getID());
//
//                // Grab critical points for zone
//                CRITICAL_POINTS cps = driveway->getCP(agent.getGZPose().zone_ID); // Grab ID of selected goal zone
//                double dist_zone = tangentialDist(gz.goal_pose, curr_pose);
//                angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
//                /*
//                 * If goal zone is not right in front of the agent
//                 * TODO: 'waypoint.x' and 'waypoint.y' may need to be set to 'curr_pose'
//                 */
//                if (gz.traverse) {
//                    double factor;
//                    double dist_cpl = tangentialDist(cps.left_CP, curr_pose);
//                    // Check if critical points along traversal are near
//                    // NOTE: I can't remember why I chose to create and check 'getReachedCPS' function
//                    if (dist_cpl < 0.2 || agent.getReachedCPS()) { // Arbitrary value; NOTE: needs to large enough to capture rover traversing away from R2
//                        /*
//                         * Change traversal path to traverse to zone garage
//                         * We need to calc equation of line depicated by critical point and where it meets RZ
//                         */
//                        // Are we at the goal zone pose
//                        if (dist_zone < 0.1) { // TODO: arbitrary value; will need to be tuned
//                            agent = driveway->moveAgent(agent, ACTIVE, GARAGE);
//                            agentMap->updateMap(agent.getID(), agent);
//                        } else if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
//                            angle = 0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
//                        } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
//                            angle = -0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
//                        } else {
//                            // Do nothing continue driving forward
//                            angle = curr_pose.theta;
//                        }
//                        velocity.linear = 0.2 * (dist_zone + (RZ / 2));
//                    } else {
//                        /*
//                         * Default traversal:
//                         * Drive till we are in the middle of driveway
//                         */
//                        if (!agent.getInitTraversal) {
//                            if (dist_anchor > CPR) {
//                                agent.setInitTraversal(true);
//                                agentMap->updateMap(agent.getID(), agent);
//                            }
//                            angle = -0.2 / dist_anchor;
//                        }
//                        else if (dist_anchor <= CPR - TRAVERSE_STD) { // Need to turn right to remain on driveway
//                            factor = CPR - TRAVERSE_STD - dist_anchor;
//                            angle = (-0.1 / dist_anchor) * factor; // TODO: arbitrary value; will need to be tuned
//
//                        } else if (dist_anchor >= CPR + TRAVERSE_STD) { // Need to left to remain on driveway
//                            factor = dist_anchor - CPR - TRAVERSE_STD;
//                            angle = (0.1 / dist_anchor) * factor; // TODO: arbitrary value; will need to be tuned
//                        } else {
//                            angle = 0.1 / dist_anchor; // Need to continually turn toward goal poze
//                        }
//                        velocity.linear = 0.2 * (dist_zone); // TODO: arbitrary value; will need to be tuned
//                    }
//
//                } else { // Goal zone is right in front
//                    if (dist_zone < 0.1) {
//                        agent = driveway->moveAgent(agent, ACTIVE, GARAGE);
//                        agentMap->updateMap(agent.getID(), agent);
//                    }
//                    if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
//                        angle = 0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
//                    } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
//                        angle = -0.025 / dist_anchor; // TODO: arbitrary value; will need to be tuned
//                    } else {
//                        // Do nothing continue driving forward
//                        angle = curr_pose.theta;
//                    }
//                    velocity.linear = 0.2 * (dist_zone + (RZ / 2));
//                }
//                velocity.angular = angle;
//                break;
//            }
//            case WAITING: {
//                if (driveway->canEnter((DRIVEWAY_TYPE) agent.getDrivewayState())) {
//                    agent = driveway->moveAgent(agent, WAITING, ACTIVE);
//                    agentMap->updateMap(agent.getID(), agent);
//                }
//                velocity.linear = 0;
//                velocity.angular = 0;
//                break;
//            }
//            // We need to make sure that even thought the rover is changing from GARAGE to DELIVERY the driveway still needs
//            // to mark that particular garage as held
//            // UPDATE: this is all handled in zoneMap since driveway and zoneMap are considered separately
//            case GARAGE:
//                // We need to turn the rover to face home
//                angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
//                if (curr_pose.theta < angle - ORIENTATION_ERR) { // Left turn; considering angles
//                    angle = 0.2 / dist_anchor; // TODO: arbitrary value; will need to be tuned
//                }
//                else if (curr_pose.theta > angle + ORIENTATION_ERR) { // Right turn; considering angles
//                    angle = -0.2 / dist_anchor; // TODO: arbitrary value; will need to be tuned
//                }
//                else {
//                    if (driveway->canEnter((DRIVEWAY_TYPE) agent.getDrivewayState())) {
//                        agent = driveway->moveAgent(agent, GARAGE, DELIVERY);
//                        agentMap->updateMap(agent.getID(), agent);
//                    }
//                }
//                velocity.linear = 0.001;
//                velocity.angular = angle;
//                break;
//            case DELIVERY: { // Traverse into home
//                /*
//                * The agent has reached the drop off point
//                * 1) Transition to DROP_OFF, which should have no properties accept to drop the cube
//                * 2) DROP_OFF passes back to after cube is released, but no movement is to be made by DROP_OFF.
//                *      DROP_OFF needs to mark that the agent no longer has a resource.
//                */
//                angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
//                if (dist_anchor < 0.2) {
//                    velocity.linear = 0.001;
//                    angle = curr_pose.theta;
//                    // Pass to DROP_OFF
//                    agent.getState()->setState(STATE_DROP_OFF);
//                    agentMap->updateMap(agent.getID(), agent);
//                } else {
//                    if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
//                        angle = 0.05 / dist_anchor;
//                    } else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
//                        angle = -0.05 / dist_anchor;
//                    } else {
//                        // Do nothing continue driving forward
//                        angle = curr_pose.theta;
//                    }
//                    velocity.linear = 0.2 * dist_anchor; // TODO: arbitrary value; will need to be tuned
//                }
//                velocity.angular = angle;
//                break;
//            }
//            default:
//                break;
//        }
//    }
//    else {
//        // TODO: we just passed from delivery (ie agent has no resource); how do we exit the driveway safely?
//        // TODO: need to set 'initTraversal' back to false after we have exited driveway
//        switch (agent.getDrivewayState()) {
//            /*
//             * Agent will be either INIT or DELIVERY
//             * -> INIT: agent default value
//             * -> DELIVERY: agent needs to leave driveway
//             */
//            case ACTIVE:
//                break;
//            case WAITING:
//                break;
//            case GARAGE:
//            break;
//            case INIT:
//                /*
//                 * Found the threshold of driveway but doesn't not have a resource.
//                 * NEED: to traverse outer boundary (R3) and make decision when to leave boundary
//                 * TODO: '0.2' is an arbitrary value and needs to be set,
//                 */
//                // TODO: need to figure out exit point; need to calculate when 'goal_search_waypoint' has a clear path
//                // TODO: exit point sets 'initTraversal' back to zero
//                if (!agent.getInitTraversal()) {
//                    if (dist_anchor > R3) { // INIT: this is the initial right turn to orient agent into traversal
//                        // We need some linear momentum otherwise we will never clear this condition for traversal
//                        agent.setInitTraversal(true);
//                        agentMap->updateMap(agent.getID(), agent);
//
//                    }
//                    angle = -0.2 / dist_anchor;
//                }
//                else if (dist_anchor > (R3 + (3 * TRAVERSE_ERR))) {
//                    angle = 0.2 / dist_anchor;
//                }
//                else if (dist_anchor < (R3 + TRAVERSE_ERR)) {
//                    // Needs to turn right to begin outer traversal
//                    // We make a right turn by holding the linear velocity constant
//                    angle = -0.2 / dist_anchor;
//                }
//                else {
//                    // Drive straight
//                    angle = curr_pose.theta;
//                }
//                velocity.linear = 0.2 * dist_anchor; // TODO: '0.2' arbitrary
//                velocity.angular = angle;
//                break;
//            case DELIVERY: // Traverse out of home
//                angle = zoneMap->angleCalc(gz.goal_pose, curr_pose);
//                if (dist_anchor > R3 + TRAVERSE_ERR) { // EXIT state <-- Need to wrap up code into different agent states
//                    agent.setInitTraversal(false);
//                    agent.getState()->setState(STATE_SEARCH);
//                    agentMap->updateMap(agent.getID(), agent);
//                }
//                else if (curr_pose.theta < angle - TRAVERSE_ERR) { // Left turn; considering angles
//                    angle = 0.025 / dist_anchor;
//                }
//                else if (curr_pose.theta > angle + TRAVERSE_ERR) { // Right turn; considering angles
//                    angle = -0.025 /dist_anchor;
//                }
//                velocity.linear = -0.2 * dist_anchor;
//                velocity.angular = angle;
//                break;
//            default:
//                break;
//        }
//    }



    int test = 0;
    VELOCITY temp = velocity;
    return 0;
}